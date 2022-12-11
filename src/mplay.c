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

#define ERR(...) err(1, __VA_ARGS__)
#define ERRX(...) errx(1, __VA_ARGS__)
#define PA_CTX_ERR(ctx, fmt, ...) do { \
		warnx(fmt ": %s", __VA_ARGS__ __VA_OPT__(,) \
			pa_strerror(pa_context_errno(ctx))); \
		pa_mainloop_quit(pa_mloop, 1); \
	} while (0)
#define PA_ERRX(...) do { \
		warnx(__VA_ARGS__); \
		pa_mainloop_quit(pa_mloop, 1); \
	} while (0)

static const pa_sample_spec pa_spec = {
	.format = PA_SAMPLE_S16LE,
	.rate = 44100,
	.channels = 2
};

static const pa_buffer_attr pa_buf = {
	.maxlength = UINT32_MAX,
	.tlength = UINT32_MAX,
	.prebuf = UINT32_MAX,
	.minreq = UINT32_MAX,
};
static pa_stream_flags_t pa_stream_flags =
	PA_STREAM_START_CORKED | PA_STREAM_INTERPOLATE_TIMING |
	PA_STREAM_NOT_MONOTONIC | PA_STREAM_AUTO_TIMING_UPDATE |
	PA_STREAM_ADJUST_LATENCY;

static pa_mainloop *pa_mloop;
static pa_context *pa_ctx;
static pa_stream *pa_strm;
static pa_sink_input_info pa_strm_info;

static struct {
	uint8_t *data;
	size_t len;
} audiofile;

static struct {
	mp3dec_t dec;
	uint8_t *pos;
	ssize_t left;
} mp3d;

static pthread_t input_worker_thread;
static pthread_t pa_worker_thread;

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

	buf = mmap(NULL, attr.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (!buf) ERR("mmap %s", path);

	*len = attr.st_size;

	close(fd);

	return buf;
}

void
pa_stream_write_callback(pa_stream *stream, size_t requested, void *data)
{
	mp3dec_frame_info_t info;
	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];
	ssize_t remaining;
	int cnt;

	remaining = requested;
	while (remaining > 0 && mp3d.left > 0) {
		cnt = mp3dec_decode_frame(&mp3d.dec,
			mp3d.pos, mp3d.left, samples, &info);
		if (!cnt && !info.frame_bytes) break;

		mp3d.pos += info.frame_bytes;
		mp3d.left -= info.frame_bytes;
		if (!cnt) continue;

		pa_stream_write(stream, samples,
			cnt * info.channels * sizeof(mp3d_sample_t),
			NULL, 0, PA_SEEK_RELATIVE);
		remaining -= info.frame_bytes;
	}
}

void
pa_stream_state_callback(pa_stream *stream, void *data)
{
	switch (pa_stream_get_state(stream)) {
	case PA_STREAM_FAILED:
		pa_mainloop_quit(pa_mloop, 1);
		return;
	case PA_STREAM_READY:
		pa_stream_cork(stream, 0, NULL, NULL);
		break;
	case PA_STREAM_UNCONNECTED:
		printf("DISCONECTED\n");
		pa_mainloop_quit(pa_mloop, 0);
		break;
	case PA_STREAM_CREATING:
	default:
		break;
	}
}

void
pa_state_callback(pa_context *pa_ctx, void *data)
{
	pa_channel_map pa_chmap;
	pa_context_state_t state;
	int ret;

	switch (pa_context_get_state(pa_ctx)) {
	case PA_CONTEXT_FAILED:
		pa_mainloop_quit(pa_mloop, 1);
		return;
	case PA_CONTEXT_READY:
		pa_channel_map_init_stereo(&pa_chmap);

		mp3dec_init(&mp3d.dec);
		mp3d.pos = audiofile.data;
		mp3d.left = audiofile.len;

		pa_strm = pa_stream_new(pa_ctx, "mplay", &pa_spec, &pa_chmap);
		if (!pa_strm) PA_CTX_ERR(pa_ctx, "pa_stream_new");

		pa_stream_set_state_callback(pa_strm,
			pa_stream_state_callback, NULL);
		pa_stream_set_write_callback(pa_strm,
			pa_stream_write_callback, NULL);

		ret = pa_stream_connect_playback(pa_strm,
			NULL, &pa_buf, pa_stream_flags, NULL, NULL);
		if (ret) PA_ERRX("pa_stream_connect_playback");

		break;
	case PA_CONTEXT_CONNECTING:
	case PA_CONTEXT_UNCONNECTED:
	default:
		break;
	}
}

void
pa_sink_input_info_callback(struct pa_context *ctx,
	const pa_sink_input_info *info, int eol, void *data)
{
	if (eol) return;
	memcpy(&pa_strm_info, info, sizeof(pa_sink_input_info));
}

void *
pa_worker(void *arg)
{
	if (pa_mainloop_run(pa_mloop, NULL))
		exitcode = 1;
	printf("KILL INPUT\n");
	pthread_kill(input_worker_thread, SIGINT);
	return NULL;
}

void *
input_worker(void *arg)
{
	pa_cvolume vol;
	struct termios old, new;
	char c;

	if (tcgetattr(0, &old))
		ERR("tcgetattr");

	new = old;
	new.c_iflag |= BRKINT;
	new.c_iflag &= ~IGNBRK;
	new.c_lflag &= ~ICANON;

	if (tcsetattr(0, TCSANOW, &new))
		ERR("tcsetattr");

	while ((c = getchar())) {
		if (c == '+') {
			pa_context_get_sink_input_info(pa_ctx,
				pa_stream_get_index(pa_strm),
				pa_sink_input_info_callback, NULL);
			pa_cvolume_inc(&pa_strm_info.volume, 1);
			pa_context_set_sink_input_volume(pa_ctx,
				pa_stream_get_index(pa_strm),
				&pa_strm_info.volume, NULL, NULL);
		} else if (c == '-') {
			pa_context_get_sink_input_info(pa_ctx,
				pa_stream_get_index(pa_strm),
				pa_sink_input_info_callback, NULL);
			pa_cvolume_dec(&pa_strm_info.volume, 1);
			pa_context_set_sink_input_volume(pa_ctx,
				pa_stream_get_index(pa_strm),
				&pa_strm_info.volume, NULL, NULL);
		}
	}

	tcsetattr(0, TCSANOW, &old);

	return NULL;
}

int
main(int argc, const char **argv)
{
	pa_mainloop_api *pa_mloop_api;
	pa_context_state_t pa_ctx_state;
	int ret;

	if (argc != 2) {
		printf("USAGE: mplay FILE\n");
		return 1;
	}

	audiofile.data = map_file(argv[1], &audiofile.len);

	pa_mloop = pa_mainloop_new();
	if (!pa_mloop) ERRX("pa_mainloop_new");

	pa_mloop_api = pa_mainloop_get_api(pa_mloop);
	if (!pa_mloop_api) ERRX("pa_mainloop_get_api");

	pa_ctx = pa_context_new(pa_mloop_api, "mplay");
	if (!pa_ctx) ERRX("pa_context_new");

	ret = pa_context_connect(pa_ctx, NULL, 0, NULL);
	if (ret) ERRX("pa_context_connect: %s",
		pa_strerror(pa_context_errno(pa_ctx)));

	pa_context_set_state_callback(pa_ctx,
		pa_state_callback, NULL);

	exitcode = 0;

	if (pthread_create(&pa_worker_thread, NULL, pa_worker, NULL))
		ERR("pthread_create");

	if (pthread_create(&input_worker_thread, NULL, input_worker, NULL))
		ERR("pthread_create");

	pthread_setschedprio(pa_worker_thread, 1);
	pthread_setschedprio(input_worker_thread, 2);

	pthread_join(input_worker_thread, NULL);
	pthread_join(pa_worker_thread, NULL);

	pa_context_disconnect(pa_ctx);
	pa_context_unref(pa_ctx);
	pa_mainloop_free(pa_mloop);

	munmap(audiofile.data, audiofile.len);

	return exitcode;
}
