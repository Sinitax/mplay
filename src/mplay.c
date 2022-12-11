#include "pulse/simple.h"
#include "pulse/volume.h"
#include "pulse/error.h"

#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <err.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static uint8_t *map_file(const char *path, size_t *len);
static void play_file(const char *path);

static const pa_sample_spec pa_spec = {
	.format = PA_SAMPLE_S16LE,
	.rate = 44100,
	.channels = 2
};

uint8_t *
map_file(const char *path, size_t *len)
{
	struct stat attr;
	uint8_t *buf;
	int fd;

	fd = open(path, O_RDONLY);
	if (fd < 0) err(1, "open %s", path);

	if (fstat(fd, &attr))
		err(1, "fstat %s", path);

	buf = mmap(NULL, attr.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (!buf) err(1, "mmap %s", path);

	*len = attr.st_size;

	close(fd);

	return buf;
}

void
play_file(const char *filename)
{
	mp3dec_frame_info_t info;
	mp3d_sample_t samples[MINIMP3_MAX_SAMPLES_PER_FRAME];
	mp3dec_t mp3d;
	pa_simple *pa;
	uint8_t *data, *pos;
	size_t len;
	int pa_err;
	int ret;
	int cnt;

	data = map_file(filename, &len);

	mp3dec_init(&mp3d);

	pa = pa_simple_new(NULL, "mplay", PA_STREAM_PLAYBACK, NULL,
		"playback", &pa_spec, NULL, NULL, &pa_err);
	if (!pa) errx(1, "pa_simple_new: %s", pa_strerror(pa_err));

	pos = data;
	while (pos < data + len) {
		cnt = mp3dec_decode_frame(&mp3d, pos, len, samples, &info);
		if (!cnt && !info.frame_bytes) break;

		pos += info.frame_bytes;
		if (!cnt) continue;

		ret = pa_simple_write(pa, samples,
			cnt * info.channels * sizeof(mp3d_sample_t), &pa_err);
		if (ret) errx(1, "pa_simple_write: %s", pa_strerror(pa_err));
	}

	ret = pa_simple_drain(pa, &pa_err);
	if (ret) errx(1, "pa_simple_drain: %s", pa_strerror(pa_err));

	pa_simple_free(pa);

	munmap(data, len);
}

int
main(int argc, const char **argv)
{
	if (argc != 2) {
		printf("USAGE: mplay FILE\n");
		exit(1);
	}

	play_file(argv[1]);
}

