#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include "pulse/error.h"
#include "pulse/simple.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static uint8_t *read_file(const char *filename, size_t *len);
static void play_file(const char *filename);
static void daemon(void);

uint8_t *
read_file(const char *filename, size_t *len)
{
	FILE *f;
	uint8_t *buf;
	size_t start;

	f = fopen(filename, "r");
	if (f == NULL)
		ERROR("Failed to open file\n");

	fseek(f, 0, SEEK_END);
	*len = ftell(f) - start;

	buf = malloc(*len);
	OOM_CHECK(buf);

	fseek(f, 0, SEEK_SET);
	if (fread(buf, 1, *len, f) != len)
		ERROR("Failed to read file contents\n");

	fclose(f);

	return buf;
}

void
play_file(const char *filename)
{
	static const pa_sample_spec spec = {
		.format = PA_SAMPLE_S16LE,
		.rate = 44100,
		.channels = 2
	};
	mp3dec_frame_info_t info;
	mp3d_sample_t sample;
	mp3dec_t dec;
	pa_simple *stream;
	uint8_t *data;
	size_t len;
	int status, ret;

	data = read_file(filename, &len);

	mp3dec_init(&dec);

	stream = pa_simple_new(NULL, "MPLAY", PA_STREAM_PLAYBACK, NULL,
		"playback", &spec, NULL, NULL, &status);
	// check status

	while (mp3dec_decode_frame(&dec, data, len, &sample, &info)) {
		pa_simple_write();
	}

	ret = pa_simple_drain(stream, &status);
	if (!ret) ERROR("Failed to drain rest of audio\n");

	pa_simple_free(stream);
}

void
daemon(void)
{
	
}

int
main(int argc, const char **argv)
{
	if (argc != 2)
		ERROR("USAGE: mplay FILE\n");

	play_file(argv[1]);
}
