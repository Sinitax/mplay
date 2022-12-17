#include <pulse/introspect.h>
#include <pulse/operation.h>
#include <pulse/stream.h>
#include <pulse/thread-mainloop.h>
#include <pulse/volume.h>
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

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

enum {
	KEY_NONE = 0,
	KEY_BRK = 3,
	KEY_QUIT = 0x100,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,
};

#define ERR(...) err(1, __VA_ARGS__)
#define ERRX(...) errx(1, __VA_ARGS__)
#define PA_ERRX(...) do { \
		warnx(__VA_ARGS__); \
		exit(1); \
	} while (0)

static pa_sample_spec pa_spec = {
	.format = PA_SAMPLE_S16LE,
	.rate = 44100,
	.channels = 2
};
static pa_buffer_attr pa_buf = {
	.fragsize = UINT32_MAX,
	.maxlength = UINT32_MAX,
	.tlength = UINT32_MAX,
	.prebuf = 0,
	.minreq = UINT32_MAX,
};
static pa_stream_flags_t pa_stream_flags =
	PA_STREAM_START_CORKED | PA_STREAM_INTERPOLATE_TIMING |
	PA_STREAM_AUTO_TIMING_UPDATE | PA_STREAM_ADJUST_LATENCY;

static pa_threaded_mainloop *pa_mloop;
static pa_context *pa_ctx;
static pa_stream *pa_strm;
static pa_sink_input_info pa_strm_info;
static bool pa_strm_info_update;

static struct {
	uint8_t *data;
	size_t len;
} audiofile;

static struct {
	mp3dec_t dec;
	uint8_t *pos;
	ssize_t left;

	mp3d_sample_t *samples;
	size_t sample_cnt;
	size_t sample_cap;

	size_t sample_next;

	bool seek;
	pa_usec_t seek_stream_time;
	pa_usec_t seek_track_time;

	int rate;
	int channels;

	bool init;
	bool pause;
	int err;
} mp3d;

void decoder_init(void);
void decoder_seek(size_t sample_pos);
int decode_next_frame(mp3d_sample_t *samples);

static pthread_t input_worker_thread;
static pthread_t pa_worker_thread;

static struct termios term_old;
static struct termios term_new;
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

	buf = mmap(NULL, attr.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (!buf) ERR("mmap %s", path);

	*len = attr.st_size;

	close(fd);

	return buf;
}

void
decoder_init(void)
{
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
	if (!decode_next_frame(samples))
		exit(1);

	mp3d.err = 0;
	mp3d.seek = false;
	mp3d.pause = false;
	mp3d.init = true;
}

int
decode_next_frame(mp3d_sample_t *samples)
{
	mp3dec_frame_info_t info;
	int cnt;

	if (mp3d.sample_next < mp3d.sample_cnt) {
		cnt = MIN(MINIMP3_MAX_SAMPLES_PER_FRAME,
			mp3d.sample_cnt - mp3d.sample_next);
		memcpy(samples, &mp3d.samples[mp3d.sample_next],
			cnt * sizeof(mp3d_sample_t));
		mp3d.sample_next += cnt;
	} else {
		cnt = mp3dec_decode_frame(&mp3d.dec,
			mp3d.pos, mp3d.left, samples, &info);
		cnt *= info.channels;

		if (mp3d.pos == audiofile.data && !cnt) {
			warnx("MP3 decode error");
			mp3d.err = 1;
			return 0;
		}

		if (!mp3d.init) {
			mp3d.rate = info.hz;
			mp3d.channels = info.channels;
		}

		if (mp3d.sample_cnt + cnt > mp3d.sample_cap) {
			mp3d.sample_cap = MAX(mp3d.sample_cap * 2,
				mp3d.sample_cnt + cnt);
			mp3d.samples = realloc(mp3d.samples,
				mp3d.sample_cap * sizeof(mp3d_sample_t));
			if (!mp3d.samples) ERR("realloc");
		}
		memcpy(&mp3d.samples[mp3d.sample_cnt], samples,
			cnt * sizeof(mp3d_sample_t));
		mp3d.sample_cnt += cnt;

		mp3d.pos += info.frame_bytes;
		mp3d.left -= info.frame_bytes;

		mp3d.sample_next += cnt;
	}

	return cnt;
}

void
decoder_seek(size_t sample_pos)
{
	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];
	int cnt;

	while (mp3d.sample_next < sample_pos) {
		cnt = decode_next_frame(samples);
		if (!cnt) exit(0);
	}
	mp3d.sample_next = sample_pos;
}

void
context_poll_until(pa_context_state_t target_state)
{
	pa_context_state_t state;

	while (true) {
		pa_threaded_mainloop_lock(pa_mloop);
		state = pa_context_get_state(pa_ctx);
		pa_threaded_mainloop_unlock(pa_mloop);
		if (state == target_state)
			break;
		pa_threaded_mainloop_wait(pa_mloop);
	}
}

void
stream_poll_until(pa_stream_state_t target_state)
{
	pa_stream_state_t state;

	while (true) {
		pa_threaded_mainloop_lock(pa_mloop);
		state = pa_stream_get_state(pa_strm);
		pa_threaded_mainloop_unlock(pa_mloop);
		if (state == target_state)
			break;
		pa_threaded_mainloop_wait(pa_mloop);
	}
}

void
update_sink_input_info_callback(struct pa_context *ctx,
	const pa_sink_input_info *info, int eol, void *data)
{
	if (eol) return;
	memcpy(&pa_strm_info, info, sizeof(pa_sink_input_info));
	pa_strm_info_update = true;
	pa_threaded_mainloop_signal(pa_mloop, 1);
}

void
update_sink_input_info(void)
{
	pa_operation *op;

	pa_strm_info_update = false;
	op = pa_context_get_sink_input_info(pa_ctx,
		pa_stream_get_index(pa_strm),
		update_sink_input_info_callback, NULL);
	if (!op) PA_ERRX("pa_context_get_sink_input_info failed");

	while (!pa_strm_info_update) {
		pa_threaded_mainloop_wait(pa_mloop);
	}

	pa_operation_unref(op);

	pa_threaded_mainloop_accept(pa_mloop);
}

void
pa_stream_write_callback(pa_stream *stream, size_t requested, void *data)
{
	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];
	const pa_timing_info *info;
	ssize_t remaining;
	int cnt, channels;

	if (!mp3d.left) {
		info = pa_stream_get_timing_info(pa_strm);
		if (info->read_index >= info->write_index)
			exit(0);
	}

	remaining = requested;
	while (remaining > 0 && mp3d.left > 0) {
		cnt = decode_next_frame(samples);
		if (!cnt) exit(mp3d.err);

		pa_stream_write(stream, samples,
			cnt * sizeof(mp3d_sample_t), NULL, 0,
			mp3d.seek ? PA_SEEK_RELATIVE_ON_READ: PA_SEEK_RELATIVE);

		mp3d.seek = false;
		remaining -= cnt * sizeof(mp3d_sample_t);
	}
}

double
get_track_time(void)
{
	pa_usec_t time;

	pa_stream_get_time(pa_strm, &time);
	return (time - mp3d.seek_stream_time + mp3d.seek_track_time) / 1000000.F;
}

ssize_t
stream_samples_buffered(void)
{
	const pa_timing_info *info;

	info = pa_stream_get_timing_info(pa_strm);
	if (info->write_index_corrupt || info->read_index_corrupt)
		return 0;

	return (info->write_index - info->read_index) / sizeof(mp3d_sample_t);
}

void
cmd_volume(float delta)
{
	pa_operation *op;

	pa_threaded_mainloop_lock(pa_mloop);

	update_sink_input_info();

	if (delta > 0) {
		pa_cvolume_inc(&pa_strm_info.volume,
			delta * PA_VOLUME_NORM / 100);
	} else {
		pa_cvolume_dec(&pa_strm_info.volume,
			-delta * PA_VOLUME_NORM / 100);
	}
	if (pa_cvolume_avg(&pa_strm_info.volume) > 2 * PA_VOLUME_NORM)
		pa_cvolume_set(&pa_strm_info.volume, 2, 2 * PA_VOLUME_NORM);

	op = pa_context_set_sink_input_volume(pa_ctx,
		pa_stream_get_index(pa_strm),
		&pa_strm_info.volume, NULL, NULL);
	pa_operation_unref(op);

	printf("VOLUME: %02.2f\n", pa_cvolume_avg(&pa_strm_info.volume)
		* 100.f / PA_VOLUME_NORM);

	pa_threaded_mainloop_unlock(pa_mloop);
}

void
cmd_seek(float delta)
{
	ssize_t sample_pos;
	pa_operation *op;
	int64_t index;

	pa_threaded_mainloop_lock(pa_mloop);

	sample_pos = MAX(0, (ssize_t) mp3d.sample_next +
		delta * mp3d.channels * mp3d.rate - stream_samples_buffered());
	decoder_seek(sample_pos);
	op = pa_stream_flush(pa_strm, NULL, NULL);
	pa_operation_unref(op);

	mp3d.seek = true;
	pa_stream_get_time(pa_strm, &mp3d.seek_stream_time);
	mp3d.seek_track_time = mp3d.sample_next * 1000000.F
		/ (mp3d.channels * mp3d.rate);

	printf("SEEK: %02.2f\n", mp3d.seek_track_time / 1000000.F);

	pa_threaded_mainloop_unlock(pa_mloop);
}

void
cmd_pause(void)
{
	ssize_t sample_pos;
	pa_operation *op;

	pa_threaded_mainloop_lock(pa_mloop);
	mp3d.pause ^= 1;
	op = pa_stream_cork(pa_strm, mp3d.pause, NULL, NULL);
	pa_operation_unref(op);
	printf("PAUSE: %i\n", mp3d.pause);
	pa_threaded_mainloop_unlock(pa_mloop);
}

void
cmd_status(void)
{
	pa_threaded_mainloop_lock(pa_mloop);
	update_sink_input_info();
	printf("STATUS: vol:%02.2f pos:%02.2f pause:%i\n",
		pa_cvolume_avg(&pa_strm_info.volume) * 100.0 / PA_VOLUME_NORM,
		get_track_time(), mp3d.pause);
	pa_threaded_mainloop_unlock(pa_mloop);
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
		return KEY_QUIT;
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

void *
input_worker(void *arg)
{
	int key;

	if (tcgetattr(0, &term_old))
		ERR("tcgetattr");
	term_set = true;

	term_new = term_old;
	term_new.c_iflag |= BRKINT;
	term_new.c_iflag &= ~IGNBRK;
	term_new.c_lflag &= ~ICANON;
	term_new.c_lflag &= ~ECHO;

	if (tcsetattr(0, TCSANOW, &term_new))
		ERR("tcsetattr");

	while (true) {
		while (!(key = getkey()));
		switch (key) {
		case KEY_UP:
		case '+':
			cmd_volume(+5);
			break;
		case KEY_DOWN:
		case '-':
			cmd_volume(-5);
			break;
		case KEY_LEFT:
			cmd_seek(-5);
			break;
		case KEY_RIGHT:
			cmd_seek(+5);
			break;
		case 'c':
			cmd_pause();
			break;
		case 's':
			cmd_status();
			break;
		case KEY_QUIT:
		case 'q':
			return NULL;
		default:
			printf("Unbound key: %i\n", key);
			break;
		}
	}

	return NULL;
}

void
context_init(void)
{
	pa_mainloop_api *pa_mloop_api;
	int ret;

	pa_threaded_mainloop_lock(pa_mloop);

	pa_mloop_api = pa_threaded_mainloop_get_api(pa_mloop);
	if (!pa_mloop_api) ERRX("pa_threaded_mainloop_get_api");

	pa_ctx = pa_context_new(pa_mloop_api, "mplay");
	if (!pa_ctx) ERRX("pa_context_new");

	ret = pa_context_connect(pa_ctx, NULL, 0, NULL);
	if (ret) ERRX("pa_context_connect: %s",
		pa_strerror(pa_context_errno(pa_ctx)));

	pa_threaded_mainloop_unlock(pa_mloop);

	context_poll_until(PA_CONTEXT_READY);
}

void
stream_init(void)
{
	pa_channel_map pa_chmap;
	pa_context_state_t state;
	int ret;

	pa_channel_map_init_stereo(&pa_chmap);

	pa_threaded_mainloop_lock(pa_mloop);

	pa_strm = pa_stream_new(pa_ctx, "mplay", &pa_spec, &pa_chmap);
	if (!pa_strm) ERRX("pa_stream_new: %s",
		pa_strerror(pa_context_errno(pa_ctx)));

	pa_stream_set_write_callback(pa_strm, pa_stream_write_callback, NULL);

	ret = pa_stream_connect_playback(pa_strm, NULL,
		&pa_buf, pa_stream_flags, NULL, NULL);
	if (ret) ERRX("pa_stream_connect_playback failed");

	pa_threaded_mainloop_unlock(pa_mloop);

	stream_poll_until(PA_STREAM_READY);
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
		tcsetattr(0, TCSANOW, &term_old);
}

int
main(int argc, const char **argv)
{
	pa_context_state_t pa_ctx_state;
	struct sigaction act;
	int ret;

	if (argc != 2) {
		printf("USAGE: mplay FILE\n");
		return 1;
	}

	audiofile.data = map_file(argv[1], &audiofile.len);
	decoder_init();

	pa_spec.channels = mp3d.channels;
	pa_spec.rate = mp3d.rate;

	pa_mloop = pa_threaded_mainloop_new();
	if (!pa_mloop) ERRX("pa_threaded_mainloop_new");

	pa_threaded_mainloop_start(pa_mloop);

	context_init();
	stream_init();

	pa_threaded_mainloop_lock(pa_mloop);
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
