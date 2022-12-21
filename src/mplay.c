#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#include <pulse/sample.h>
#include <pulse/introspect.h>
#include <pulse/operation.h>
#include <pulse/stream.h>
#include <pulse/thread-mainloop.h>
#include <pulse/volume.h>
#include <pulse/pulseaudio.h>
#include <pulse/def.h>

#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <err.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define ERR(...) err(1, __VA_ARGS__)
#define ERRX(...) errx(1, __VA_ARGS__)

enum {
	KEY_NONE = 0,
	KEY_BRK = 3,
	KEY_EOF = 0x100,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,
};

enum {
	CMD_QUIT
};

struct audiofile {
	uint8_t *data;
	size_t len;
};

struct mp3d {
	mp3dec_t dec;
	uint8_t *pos;
	ssize_t left;

	mp3d_sample_t *samples;
	size_t sample_cnt;
	size_t sample_cap;

	size_t sample_next;

	bool seek;

	int rate;
	int channels;

	bool init;
	bool pause;
};

void decoder_init(void);
void decoder_seek(size_t sample_pos);
int decode_next_frame(mp3d_sample_t *samples, int *size);

static pa_sample_spec pa_spec = {
	.format = PA_SAMPLE_S16LE,
	.rate = 44100,
	.channels = 2
};

static pa_buffer_attr pa_buf = {
	.fragsize = UINT32_MAX,
	.maxlength = UINT32_MAX,
	.tlength = UINT32_MAX,
	.prebuf = UINT32_MAX,
	.minreq = UINT32_MAX,
};

static pa_stream_flags_t pa_stream_flags = PA_STREAM_START_CORKED
	| PA_STREAM_AUTO_TIMING_UPDATE | PA_STREAM_ADJUST_LATENCY;

static pa_threaded_mainloop *pa_mloop;
static pa_context *pa_ctx;
static pa_stream *pa_strm;
static pa_sink_input_info pa_strm_sink;
static bool pa_strm_sink_update;

static bool cmd_input_mode = false;

static struct audiofile audiofile = { 0 };

static struct mp3d mp3d = { 0 };

static pthread_t input_worker_thread;

static struct termios term_orig;
static struct termios term_raw;
static bool term_set = false;

static int exitcode;

uint8_t *
map_file(const char *path, size_t *len)
{
	struct stat attr;
	uint8_t *buf;
	int fd;

	fd = open(path, O_RDONLY);
	if (fd < 0) ERR("open %s", path);

	if (fstat(fd, &attr))
		ERR("fstat %s", path);

	if ((attr.st_mode & S_IFMT) == S_IFDIR)
		ERRX("not a file: %s", path);

	buf = mmap(NULL, attr.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (!buf) ERR("mmap %s", path);

	*len = attr.st_size;

	close(fd);

	return buf;
}

void
decoder_init(void)
{
	int size;

	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];

	mp3dec_init(&mp3d.dec);
	mp3d.pos = audiofile.data;
	mp3d.left = audiofile.len;

	mp3d.sample_cnt = 0;
	mp3d.sample_next = 0;
	mp3d.sample_cap = MINIMP3_MAX_SAMPLES_PER_FRAME;
	mp3d.samples = malloc(mp3d.sample_cap * sizeof(mp3d_sample_t));
	if (!mp3d.samples) ERR("malloc");

	/* decode channel specs */
	decode_next_frame(samples, &size);
	if (!size) {
		printf("+EXIT: Invalid mp3\n");
		exit(1);
	}

	mp3d.seek = false;
	mp3d.pause = false;
	mp3d.init = true;
}

int
decode_next_frame(mp3d_sample_t *samples, int *size)
{
	static const int max_cnt = MINIMP3_MAX_SAMPLES_PER_FRAME;
	mp3dec_frame_info_t info;
	int cnt;

	if (size) *size = 0;

	if (mp3d.sample_next < mp3d.sample_cnt) {
		cnt = MIN(max_cnt, mp3d.sample_cnt - mp3d.sample_next);
		*size = cnt * sizeof(mp3d_sample_t);
	} else {
		if (mp3d.sample_cnt + max_cnt > mp3d.sample_cap) {
			mp3d.sample_cap = MAX(mp3d.sample_cap * 2,
				mp3d.sample_cnt + max_cnt);
			mp3d.samples = realloc(mp3d.samples,
				mp3d.sample_cap * sizeof(mp3d_sample_t));
			if (!mp3d.samples) ERR("realloc");
		}

		cnt = mp3dec_decode_frame(&mp3d.dec, mp3d.pos, mp3d.left,
			&mp3d.samples[mp3d.sample_cnt], &info);
		cnt *= info.channels;
		mp3d.sample_cnt += cnt;

		if (size) *size = info.frame_bytes;

		if (!info.frame_bytes)
			return 0;

		if (!mp3d.init) {
			mp3d.rate = info.hz;
			mp3d.channels = info.channels;
		}

		mp3d.pos += info.frame_bytes;
		mp3d.left -= info.frame_bytes;
	}

	if (samples != NULL)
		memcpy(samples, &mp3d.samples[mp3d.sample_next],
			cnt * sizeof(mp3d_sample_t));
	mp3d.sample_next += cnt;

	return cnt;
}

void
decoder_seek(size_t sample_pos)
{
	int size, cnt;

	while (mp3d.sample_next < sample_pos) {
		cnt = decode_next_frame(NULL, &size);
		if (!size) {
			printf("+EXIT: End of file (seek)\n");
			exit(0);
		}
	}
	mp3d.sample_next = sample_pos;
}

void
pa_stream_write_callback(pa_stream *stream, size_t requested, void *data)
{
	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];
	ssize_t remaining;
	int cnt, size;

	remaining = requested;
	while (remaining > 0) {
		cnt = decode_next_frame(samples, &size);
		if (!size) {
			printf("+EXIT: End of file\n");
			exit(0);
		}
		if (!cnt) continue;

		pa_stream_write(stream, samples,
			cnt * sizeof(mp3d_sample_t), NULL, 0,
			mp3d.seek ? PA_SEEK_RELATIVE_ON_READ: PA_SEEK_RELATIVE);

		mp3d.seek = false;
		remaining -= cnt * sizeof(mp3d_sample_t);
	}
}
void

update_sink_input_info_callback(struct pa_context *ctx,
	const pa_sink_input_info *info, int eol, void *data)
{
	if (eol) return;
	memcpy(&pa_strm_sink, info, sizeof(pa_sink_input_info));
	pa_strm_sink_update = true;
	pa_threaded_mainloop_signal(pa_mloop, true);
}

void
update_sink_input_info(void)
{
	pa_operation *op;

	pa_strm_sink_update = false;
	op = pa_context_get_sink_input_info(pa_ctx,
		pa_stream_get_index(pa_strm),
		update_sink_input_info_callback, NULL);
	if (!op) ERRX("pa_context_get_sink_input_info failed");

	while (!pa_strm_sink_update) {
		pa_threaded_mainloop_unlock(pa_mloop);
		pa_threaded_mainloop_wait(pa_mloop);
		pa_threaded_mainloop_lock(pa_mloop);
	}

	pa_operation_unref(op);

	pa_threaded_mainloop_accept(pa_mloop);
}

ssize_t
stream_samples_buffered(void)
{
	static size_t last_read_index = 0, last_write_index = 0;
	const pa_timing_info *info;
	size_t buffered;

	info = pa_stream_get_timing_info(pa_strm);
	if (!info || info->write_index_corrupt || info->read_index_corrupt)
		return 0;

	if (info->read_index != last_read_index)
		last_write_index = info->write_index;

	buffered = (last_write_index - last_read_index)
		/ sizeof(mp3d_sample_t);

	last_read_index = info->read_index;

	return buffered;
}

double
user_vol(void)
{
	pa_volume_t vol;

	update_sink_input_info();
	if (!pa_strm_sink.has_volume)
		return -1;

	vol = pa_cvolume_avg(&pa_strm_sink.volume);

	return vol * 100.F / PA_VOLUME_NORM;
}

double
user_time(void)
{
	ssize_t sample_pos;

	sample_pos = MAX(0, mp3d.sample_next - stream_samples_buffered());

	return sample_pos * 1.F / (mp3d.rate * mp3d.channels);
}

void
cmd_setvol(double vol)
{
	pa_operation *op;

	update_sink_input_info();
	pa_cvolume_set(&pa_strm_sink.volume, 2, vol * PA_VOLUME_NORM / 100.F);
	if (pa_cvolume_avg(&pa_strm_sink.volume) > 2 * PA_VOLUME_NORM)
		pa_cvolume_set(&pa_strm_sink.volume, 2, 2 * PA_VOLUME_NORM);

	op = pa_context_set_sink_input_volume(pa_ctx,
		pa_stream_get_index(pa_strm),
		&pa_strm_sink.volume, NULL, NULL);
	pa_operation_unref(op);

	printf("+VOLUME: %02.2f\n", pa_cvolume_avg(&pa_strm_sink.volume)
		* 100.f / PA_VOLUME_NORM);
}

void
cmd_seek(double time_sec)
{
	ssize_t sample_pos;
	pa_operation *op;

	if (time_sec < 0) time_sec = 0;
	sample_pos = time_sec * mp3d.channels * mp3d.rate;
	decoder_seek(sample_pos);

	op = pa_stream_flush(pa_strm, NULL, NULL);
	pa_operation_unref(op);
	mp3d.seek = true;

	printf("+SEEK: %02.2f\n", user_time());
}

void
cmd_pause_toggle(void)
{
	pa_operation *op;

	mp3d.pause ^= 1;
	op = pa_stream_cork(pa_strm, mp3d.pause, NULL, NULL);
	pa_operation_unref(op);

	printf("+PAUSE: %i\n", mp3d.pause);
}

void
cmd_status(void)
{
	update_sink_input_info();

	printf("+STATUS: vol:%02.2f pos:%02.2f pause:%i\n",
		user_vol(), user_time(), mp3d.pause);
}

int
getkey_esc(void)
{
	int c;

	c = getchar();
	switch (c) {
	case 'A':
		return KEY_UP;
	case 'B':
		return KEY_DOWN;
	case 'C':
		return KEY_RIGHT;
	case 'D':
		return KEY_LEFT;
	case EOF:
		return KEY_EOF;
	default:
		return KEY_NONE;
	}
}

int
getkey(void)
{
	int c;

	c = getchar();
	switch (c) {
	case 0x1b:
		c = getchar();
		switch (c) {
		case '[':
			return getkey_esc();
		default:
			return KEY_NONE;
		}
	default:
		return c & 0xff;
	}
}

bool
cmd_input(void)
{
	char linebuf[256];
	char *end, *tok;
	float input;

	if (!fgets(linebuf, sizeof(linebuf), stdin))
		return false;

	tok = strchr(linebuf, '\n');
	if (tok) *tok = '\0';

	pa_threaded_mainloop_lock(pa_mloop);

	if (!strncmp(linebuf, "seek ", 5)) {
		input = strtof(linebuf + 5, &end);
		if (!end || end && *end) {
			printf("Invalid input\n");
			goto exit;
		}
		cmd_seek(input);
	} else if (!strncmp(linebuf, "vol ", 4)) {
		input = strtof(linebuf + 4, &end);
		if (!end || end && *end) {
			printf("Invalid input\n");
			goto exit;
		}
		cmd_setvol(input);
	} else if (!strcmp(linebuf, "status")) {
		cmd_status();
	} else if (!strcmp(linebuf, "pause")) {
		cmd_pause_toggle();
	} else if (!strcmp(linebuf, "key")) {
		cmd_input_mode = false;
		if (term_set)
			tcsetattr(0, TCSANOW, &term_raw);
		printf("+CMDINPUT: 0\n");
	} else {
		printf("Invalid command\n");
	}

exit:
	pa_threaded_mainloop_unlock(pa_mloop);
	return true;
}

bool
key_input(void)
{
	int key;

	while (!(key = getkey()));
	if (key == 'q' || key == KEY_EOF)
		return false;

	pa_threaded_mainloop_lock(pa_mloop);

	switch (key) {
	case KEY_UP:
	case '+':
		cmd_setvol(user_vol() + 5);
		break;
	case KEY_DOWN:
	case '-':
		cmd_setvol(user_vol() - 5);
		break;
	case KEY_LEFT:
		cmd_seek(user_time() - 5);
		break;
	case KEY_RIGHT:
		cmd_seek(user_time() + 5);
		break;
	case 'c':
		cmd_pause_toggle();
		break;
	case 's':
		cmd_status();
		break;
	case 'i':
		cmd_input_mode = true;
		printf("+CMDINPUT: 1\n");
		if (term_set)
			tcsetattr(0, TCSANOW, &term_orig);
		break;
	default:
		printf("Unbound key: %i\n", key);
		break;
	}

	pa_threaded_mainloop_unlock(pa_mloop);

	return true;
}

void *
input_worker(void *arg)
{
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);

	if (!tcgetattr(0, &term_orig))
		term_set = true;

	term_raw = term_orig;
	term_raw.c_iflag |= BRKINT;
	term_raw.c_iflag &= ~IGNBRK;
	term_raw.c_lflag &= ~ICANON;
	term_raw.c_lflag &= ~ECHO;

	if (!cmd_input_mode) {
		if (tcsetattr(0, TCSANOW, &term_raw))
			ERR("tcsetattr");
	}

	printf("+READY\n");

	while (true) {
		if (cmd_input_mode) {
			if (!cmd_input())
				break;
		} else {
			if (!key_input())
				break;
		}
	}

	return NULL;
}

void
pulse_context_init(void)
{
	pa_mainloop_api *pa_mloop_api;
	int ret;

	pa_mloop_api = pa_threaded_mainloop_get_api(pa_mloop);
	if (!pa_mloop_api) ERRX("pa_threaded_mainloop_get_api");

	pa_ctx = pa_context_new(pa_mloop_api, "mplay");
	if (!pa_ctx) ERRX("pa_context_new");

	ret = pa_context_connect(pa_ctx, NULL, 0, NULL);
	if (ret) ERRX("pa_context_connect: %s",
		pa_strerror(pa_context_errno(pa_ctx)));

	while (pa_context_get_state(pa_ctx) != PA_CONTEXT_READY) {
		pa_threaded_mainloop_unlock(pa_mloop);
		pa_threaded_mainloop_wait(pa_mloop);
		pa_threaded_mainloop_lock(pa_mloop);
	}
}

void
pulse_stream_init(void)
{
	pa_channel_map pa_chmap;
	int ret;

	pa_channel_map_init_stereo(&pa_chmap);

	pa_strm = pa_stream_new(pa_ctx, "mplay", &pa_spec, &pa_chmap);
	if (!pa_strm) ERRX("pa_stream_new: %s",
		pa_strerror(pa_context_errno(pa_ctx)));

	pa_stream_set_write_callback(pa_strm, pa_stream_write_callback, NULL);

	ret = pa_stream_connect_playback(pa_strm, NULL,
		&pa_buf, pa_stream_flags, NULL, NULL);
	if (ret) ERRX("pa_stream_connect_playback failed");

	while (pa_stream_get_state(pa_strm) != PA_STREAM_READY) {
		pa_threaded_mainloop_unlock(pa_mloop);
		pa_threaded_mainloop_wait(pa_mloop);
		pa_threaded_mainloop_lock(pa_mloop);
	}
}

void
sigint_handler(int sig)
{
	exit(0);
}

void
term_reset(void)
{
	if (term_set)
		tcsetattr(0, TCSANOW, &term_orig);
}

void
usage(void)
{
	fprintf(stderr, "Usage: mplay [-i] FILE\n");
	exit(1);
}

int
main(int argc, const char **argv)
{
	const char **arg;
	const char *file;

	file = NULL;
	for (arg = argv + 1; *arg; arg++) {
		if (!strcmp(*arg, "-i")) {
			cmd_input_mode = true;
		} else {
			if (file) usage();
			file = *arg;
		}
	}

	if (!file) usage();

	audiofile.data = map_file(file, &audiofile.len);
	decoder_init();

	pa_spec.channels = mp3d.channels;
	pa_spec.rate = mp3d.rate;

	pa_mloop = pa_threaded_mainloop_new();
	if (!pa_mloop) ERRX("pa_threaded_mainloop_new");

	pa_threaded_mainloop_start(pa_mloop);

	pa_threaded_mainloop_lock(pa_mloop);
	pulse_context_init();
	pulse_stream_init();
	pa_stream_cork(pa_strm, 0, NULL, NULL);
	pa_threaded_mainloop_unlock(pa_mloop);

	exitcode = 0;
	atexit(term_reset);
	signal(SIGINT, sigint_handler);

	if (pthread_create(&input_worker_thread, NULL, input_worker, NULL))
		ERR("pthread_create");
	pthread_join(input_worker_thread, NULL);

	pa_threaded_mainloop_stop(pa_mloop);

	pa_stream_disconnect(pa_strm);
	pa_stream_unref(pa_strm);

	pa_context_disconnect(pa_ctx);
	pa_context_unref(pa_ctx);

	pa_threaded_mainloop_free(pa_mloop);

	munmap(audiofile.data, audiofile.len);

	return exitcode;
}
