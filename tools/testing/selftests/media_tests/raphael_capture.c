// SPDX-License-Identifier: GPL-2.0
/*
 * Narrow V4L2 evidence exporter for later, explicitly authorized device tests.
 * Uses the installed userspace videodev2.h, not generated kernel headers.
 * One contiguous memory plane, packed Bayer RAW10 or NV12/NV21 only.
 *
 * Build independently, never through the kernel build:
 *   cc -std=c11 -O2 -Wall -Wextra -Werror raphael_capture.c -o /tmp/raphael_capture
 * --help opens no device. A normal invocation DOES configure and stream the
 * explicitly named video device; media links and sensor setup are external.
 * Inspect the exported frames and capture.json after capture. This exporter
 * never claims that completion proves correct pixels or color quality.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#define MAX_BUFFERS 32
#define MAX_FRAMES 60
#define BUFFER_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE

struct options {
	const char *device, *camera, *route, *output;
	uint32_t format, width, height, frames, timeout_ms, buffers;
};

struct mapping {
	void *address;
	size_t length;
};

/* POSIX signal-handler communication requires volatile sig_atomic_t. */
static volatile sig_atomic_t interrupted;

static void on_signal(int signal_number)
{
	interrupted = signal_number;
}

static void usage(FILE *out)
{
	fputs(
		"Usage: raphael_capture --device /dev/videoN --camera LABEL --route LABEL\n"
		"  --output NEW_DIRECTORY --format FOURCC --width N --height N\n"
		"  [--frames 30..60] [--buffers 2..32] [--timeout-ms 100..60000]\n"
		"\n"
		"Defaults: 60 frames, 4 MMAP buffers, 5000 ms per dequeue deadline.\n"
		"Formats: pRAA, pgAA, pGAA, pBAA (packed RAW10), NV12, NV21.\n"
		"Requires VIDEO_CAPTURE_MPLANE + STREAMING and exactly one contiguous\n"
		"memory plane. NV12M/NV21M, unpacked RAW10 and tiled formats are rejected.\n"
		"\n"
		"WARNING: a normal invocation opens, configures and streams the named\n"
		"device. Run only with hardware authorization and correct media links.\n"
		"--help opens no device. No shell commands or media-link changes occur.\n"
		"\n"
		"The driver may adjust dimensions; capture.json records VIDIOC_G_FMT.\n"
		"A different returned pixel format or interlaced field is rejected.\n"
		"Each frame-NNN.bin contains precisely DQBUF bytesused bytes from offset\n"
		"zero, including data_offset and row padding. JSON retains actual index,\n"
		"sequence, timeval/timestamp_ns, full flags, bytesused, data_offset and\n"
		"QUERYBUF buffer_length. ERROR buffers are saved, never skipped.\n"
		"New output directory required; existing files are never overwritten.\n"
		"Interruptions/errors leave incomplete evidence rather than a pass.\n"
		"\n"
		"Synchronous saving can starve the capture queue. Use fast storage/tmpfs,\n"
		"compare frame timestamps with the sensor cadence, and retain driver logs.\n"
		"60 DQBUFs alone do not prove that no sensor frames were dropped.\n"
		"Inspect exported frames and capture.json after capture.\n"
		"Repeat independently per camera and RAW10/PIX NV12 or NV21 route.\n"
		"A full capture is NOT hardware/color acceptance. Check sequences,\n"
		"timestamps, errors, payloads, moving targets and a known color chart.\n"
		"Exit: 0 = requested frames exported and clean shutdown (not acceptance),\n"
		"1 = capture/save/cleanup error or ERROR buffers, 2 = invalid arguments.\n",
		out);
}

static bool accepted_format(uint32_t format)
{
	switch (format) {
	case V4L2_PIX_FMT_SRGGB10P:
	case V4L2_PIX_FMT_SGRBG10P:
	case V4L2_PIX_FMT_SGBRG10P:
	case V4L2_PIX_FMT_SBGGR10P:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return true;
	default:
		return false;
	}
}

static bool parse_number(const char *text, uint32_t low, uint32_t high,
			 uint32_t *result)
{
	char *end;
	unsigned long long value;

	if (!text[0] || text[0] == '-')
		return false;
	errno = 0;
	value = strtoull(text, &end, 10);
	if (errno || *end || value < low || value > high)
		return false;
	*result = (uint32_t)value;
	return true;
}

static int parse_options(int argc, char **argv, struct options *opt)
{
	static const struct option options[] = {
		{ "help", no_argument, NULL, 'h' },
		{ "device", required_argument, NULL, 'd' },
		{ "camera", required_argument, NULL, 'c' },
		{ "route", required_argument, NULL, 'r' },
		{ "output", required_argument, NULL, 'o' },
		{ "format", required_argument, NULL, 'f' },
		{ "width", required_argument, NULL, 'w' },
		{ "height", required_argument, NULL, 'H' },
		{ "frames", required_argument, NULL, 'n' },
		{ "buffers", required_argument, NULL, 'b' },
		{ "timeout-ms", required_argument, NULL, 't' },
		{ NULL, 0, NULL, 0 }
	};
	int option;

	opt->frames = 60;
	opt->buffers = 4;
	opt->timeout_ms = 5000;
	while ((option = getopt_long(argc, argv, "", options, NULL)) != -1) {
		switch (option) {
		case 'h':
			usage(stdout);
			return 1;
		case 'd':
			opt->device = optarg;
			break;
		case 'c':
			opt->camera = optarg;
			break;
		case 'r':
			opt->route = optarg;
			break;
		case 'o':
			opt->output = optarg;
			break;
		case 'f':
			if (strlen(optarg) != 4)
				return -1;
			opt->format = v4l2_fourcc((uint8_t)optarg[0],
						 (uint8_t)optarg[1],
						 (uint8_t)optarg[2],
						 (uint8_t)optarg[3]);
			break;
		case 'w':
			if (!parse_number(optarg, 1, UINT32_MAX, &opt->width))
				return -1;
			break;
		case 'H':
			if (!parse_number(optarg, 1, UINT32_MAX, &opt->height))
				return -1;
			break;
		case 'n':
			if (!parse_number(optarg, 30, MAX_FRAMES, &opt->frames))
				return -1;
			break;
		case 'b':
			if (!parse_number(optarg, 2, MAX_BUFFERS, &opt->buffers))
				return -1;
			break;
		case 't':
			if (!parse_number(optarg, 100, 60000, &opt->timeout_ms))
				return -1;
			break;
		default:
			return -1;
		}
	}
	if (optind != argc || !opt->device || !*opt->device ||
	    !opt->camera || !*opt->camera || !opt->route || !*opt->route ||
	    !opt->output || !*opt->output || !opt->width || !opt->height ||
	    !accepted_format(opt->format))
		return -1;
	return 0;
}

static int xioctl(int fd, unsigned long request, void *argument)
{
	int result;

	do {
		result = ioctl(fd, request, argument);
	} while (result < 0 && errno == EINTR && !interrupted);
	return result;
}

static void json_string(FILE *out, const char *text)
{
	const unsigned char *p = (const unsigned char *)text;

	fputc('"', out);
	for (; *p; p++) {
		if (*p == '"' || *p == '\\')
			fprintf(out, "\\%c", *p);
		else if (*p < 0x20)
			fprintf(out, "\\u%04x", *p);
		else
			fputc(*p, out);
	}
	fputc('"', out);
}

static int64_t monotonic_ms(void)
{
	struct timespec now;

	if (clock_gettime(CLOCK_MONOTONIC, &now) < 0)
		return -1;
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int dequeue(int fd, struct v4l2_buffer *buffer,
		   struct v4l2_plane *plane, uint32_t timeout_ms)
{
	int64_t start = monotonic_ms(), deadline;
	struct pollfd event = { .fd = fd, .events = POLLIN };

	if (start < 0)
		return -1;
	deadline = start + timeout_ms;
	for (;;) {
		int64_t now, remaining;
		int ready;

		if (interrupted) {
			errno = EINTR;
			return -1;
		}
		now = monotonic_ms();
		if (now < 0)
			return -1;
		remaining = deadline - now;
		if (remaining <= 0) {
			errno = ETIMEDOUT;
			return -1;
		}
		ready = poll(&event, 1, (int)remaining);
		if (ready < 0 && errno == EINTR)
			continue;
		if (ready < 0)
			return -1;
		if (!ready) {
			errno = ETIMEDOUT;
			return -1;
		}
		if (event.revents & (POLLNVAL | POLLHUP)) {
			errno = ENODEV;
			return -1;
		}
		if (!(event.revents & POLLIN)) {
			if (event.revents & POLLERR) {
				errno = EIO;
				return -1;
			}
			continue;
		}
		memset(buffer, 0, sizeof(*buffer));
		memset(plane, 0, sizeof(*plane));
		buffer->type = BUFFER_TYPE;
		buffer->memory = V4L2_MEMORY_MMAP;
		buffer->length = 1;
		buffer->m.planes = plane;
		if (xioctl(fd, VIDIOC_DQBUF, buffer) == 0)
			return 0;
		if (errno != EAGAIN && errno != EINTR)
			return -1;
	}
}

static int queue(int fd, uint32_t index, size_t length)
{
	struct v4l2_plane plane = { .length = (uint32_t)length };
	struct v4l2_buffer buffer = {
		.type = BUFFER_TYPE, .memory = V4L2_MEMORY_MMAP,
		.index = index, .length = 1, .m.planes = &plane
	};

	return xioctl(fd, VIDIOC_QBUF, &buffer);
}

static bool valid_layout(const struct v4l2_pix_format_mplane *pix)
{
	bool raw = pix->pixelformat != V4L2_PIX_FMT_NV12 &&
		   pix->pixelformat != V4L2_PIX_FMT_NV21;
	uint64_t row_bytes, rows;

	if (!accepted_format(pix->pixelformat) || pix->num_planes != 1 ||
	    pix->field != V4L2_FIELD_NONE || !pix->width || !pix->height ||
	    pix->height % 2 || pix->width % (raw ? 4 : 2))
		return false;
	row_bytes = raw ? (uint64_t)pix->width * 5 / 4 : pix->width;
	rows = raw ? pix->height : (uint64_t)pix->height * 3 / 2;
	return pix->plane_fmt[0].bytesperline >= row_bytes &&
	       rows <= pix->plane_fmt[0].sizeimage / pix->plane_fmt[0].bytesperline;
}

static void manifest_header(FILE *out, const struct options *opt,
			    const struct v4l2_pix_format_mplane *pix)
{
	char code[5] = {
		pix->pixelformat & 0xff, (pix->pixelformat >> 8) & 0xff,
		(pix->pixelformat >> 16) & 0xff, (pix->pixelformat >> 24) & 0xff, 0
	};

	fputs("{\n  \"camera\":", out);
	json_string(out, opt->camera);
	fputs(",\n  \"route\":", out);
	json_string(out, opt->route);
	fputs(",\n  \"device\":", out);
	json_string(out, opt->device);
	fputs(",\n  \"format\":", out);
	json_string(out, code);
	fprintf(out,
		",\n  \"width\":%" PRIu32 ", \"height\":%" PRIu32
		", \"bytesperline\":%" PRIu32 ", \"sizeimage\":%" PRIu32 ",\n"
		"  \"num_planes\":%u, \"field\":%" PRIu32 ",\n"
		"  \"colorspace\":%" PRIu32 ", \"ycbcr_enc\":%u,"
		" \"quantization\":%u, \"xfer_func\":%u,\n"
		"  \"requested_frames\":%" PRIu32 ",\n  \"frames\":[\n",
		pix->width, pix->height, pix->plane_fmt[0].bytesperline,
		pix->plane_fmt[0].sizeimage, pix->num_planes, pix->field,
		pix->colorspace, pix->ycbcr_enc, pix->quantization, pix->xfer_func,
		opt->frames);
}

static bool timestamp_ns(const struct timeval *time, uint64_t *ns)
{
	uint64_t micros;

	if (time->tv_sec < 0 || time->tv_usec < 0 || time->tv_usec >= 1000000)
		return false;
	micros = (uint64_t)time->tv_usec * 1000;
	if ((uint64_t)time->tv_sec > (UINT64_MAX - micros) / 1000000000)
		return false;
	*ns = (uint64_t)time->tv_sec * 1000000000 + micros;
	return true;
}

static void manifest_frame(FILE *out, uint32_t row, const char *filename,
			   const struct v4l2_buffer *buffer,
			   const struct v4l2_plane *plane, size_t length,
			   const char *error)
{
	uint64_t ns;
	bool valid_time = timestamp_ns(&buffer->timestamp, &ns);

	if (row)
		fputs(",\n", out);
	fputs("    {\"file\":", out);
	if (filename)
		json_string(out, filename);
	else
		fputs("null", out);
	fprintf(out,
		", \"sequence\":%" PRIu32 ", \"flags\":\"0x%08" PRIx32 "\""
		", \"buffer_index\":%" PRIu32 ", \"field\":%" PRIu32
		", \"bytesused\":%" PRIu32 ", \"data_offset\":%" PRIu32
		", \"buffer_length\":%zu, \"timestamp_sec\":%jd,"
		" \"timestamp_usec\":%jd, \"timestamp_ns\":",
		buffer->sequence, buffer->flags, buffer->index, buffer->field,
		plane->bytesused, plane->data_offset, length,
		(intmax_t)buffer->timestamp.tv_sec, (intmax_t)buffer->timestamp.tv_usec);
	if (valid_time)
		fprintf(out, "%" PRIu64, ns);
	else
		fputs("null", out);
	if (error) {
		fputs(", \"capture_error\":", out);
		json_string(out, error);
	}
	fputc('}', out);
}

static int save_frame(int dirfd, const char *name, const void *address, size_t bytes)
{
	const unsigned char *data = address;
	size_t written = 0;
	int fd = openat(dirfd, name, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);

	if (fd < 0)
		return -1;
	while (written < bytes) {
		ssize_t count;

		if (interrupted) {
			errno = EINTR;
			goto fail;
		}
		count = write(fd, data + written, bytes - written);
		if (count < 0 && errno == EINTR)
			continue;
		if (count <= 0) {
			if (!count)
				errno = EIO;
			goto fail;
		}
		written += (size_t)count;
	}
	return close(fd);
fail:
	{
		int saved = errno;

		close(fd);
		errno = saved;
		return -1;
	}
}

static void failure(char *error, size_t size, const char *stage)
{
	int saved = errno;

	fprintf(stderr, "%s: %s\n", stage, strerror(saved));
	/* Keep the first failure; later cleanup failures still reach stderr. */
	if (!error[0])
		snprintf(error, size, "%s: %s", stage, strerror(saved));
}

int main(int argc, char **argv)
{
	struct options opt = { 0 };
	struct mapping maps[MAX_BUFFERS] = { 0 };
	struct v4l2_capability capability = { 0 };
	struct v4l2_format format = { .type = BUFFER_TYPE };
	struct v4l2_requestbuffers request = {
		.type = BUFFER_TYPE, .memory = V4L2_MEMORY_MMAP
	};
	struct sigaction handler = { .sa_handler = on_signal };
	enum v4l2_buf_type type = BUFFER_TYPE;
	struct stat device_stat;
	FILE *manifest = NULL;
	char error[256] = "";
	int fd = -1, dirfd = -1, manifest_fd = -1, result;
	uint32_t count = 0, records = 0, error_buffers = 0, capabilities;
	bool requested = false, streaming = false, completed = false;

	result = parse_options(argc, argv, &opt);
	if (result > 0)
		return 0;
	if (result < 0) {
		usage(stderr);
		return 2;
	}
	sigemptyset(&handler.sa_mask);
	if (sigaction(SIGINT, &handler, NULL) < 0 ||
	    sigaction(SIGTERM, &handler, NULL) < 0) {
		perror("sigaction");
		return 1;
	}
	fd = open(opt.device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		failure(error, sizeof(error), "open video device");
		goto cleanup;
	}
	if (fstat(fd, &device_stat) < 0) {
		failure(error, sizeof(error), "stat video device");
		goto cleanup;
	}
	if (!S_ISCHR(device_stat.st_mode)) {
		errno = ENODEV;
		failure(error, sizeof(error), "video path is not a character device");
		goto cleanup;
	}
	if (xioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
		failure(error, sizeof(error), "VIDIOC_QUERYCAP");
		goto cleanup;
	}
	capabilities = capability.capabilities & V4L2_CAP_DEVICE_CAPS ?
		       capability.device_caps : capability.capabilities;
	if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) ||
	    !(capabilities & V4L2_CAP_STREAMING)) {
		errno = ENOTSUP;
		failure(error, sizeof(error), "requires MPLANE capture and streaming");
		goto cleanup;
	}
	format.fmt.pix_mp.width = opt.width;
	format.fmt.pix_mp.height = opt.height;
	format.fmt.pix_mp.pixelformat = opt.format;
	format.fmt.pix_mp.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
		failure(error, sizeof(error), "VIDIOC_S_FMT");
		goto cleanup;
	}
	memset(&format, 0, sizeof(format));
	format.type = BUFFER_TYPE;
	if (xioctl(fd, VIDIOC_G_FMT, &format) < 0) {
		failure(error, sizeof(error), "VIDIOC_G_FMT");
		goto cleanup;
	}
	if (format.fmt.pix_mp.pixelformat != opt.format ||
	    !valid_layout(&format.fmt.pix_mp)) {
		errno = ENOTSUP;
		failure(error, sizeof(error), "unsupported returned format/layout");
		goto cleanup;
	}
	if (mkdir(opt.output, 0700) < 0) {
		failure(error, sizeof(error), "create NEW output directory");
		goto cleanup;
	}
	dirfd = open(opt.output, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	if (dirfd < 0) {
		failure(error, sizeof(error), "open output directory");
		goto cleanup;
	}
	manifest_fd = openat(dirfd, "capture.json",
			     O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
	if (manifest_fd < 0) {
		failure(error, sizeof(error), "create capture.json");
		goto cleanup;
	}
	manifest = fdopen(manifest_fd, "w");
	if (!manifest) {
		failure(error, sizeof(error), "fdopen capture.json");
		goto cleanup;
	}
	manifest_fd = -1;
	manifest_header(manifest, &opt, &format.fmt.pix_mp);
	if (fflush(manifest) != 0) {
		failure(error, sizeof(error), "write capture header");
		goto cleanup;
	}
	request.count = opt.buffers;
	if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0) {
		failure(error, sizeof(error), "VIDIOC_REQBUFS");
		goto cleanup;
	}
	requested = true;
	if (request.count < 2 || request.count > MAX_BUFFERS) {
		errno = ENOBUFS;
		failure(error, sizeof(error), "unsupported returned buffer count");
		goto cleanup;
	}
	for (uint32_t i = 0; i < request.count; i++) {
		struct v4l2_plane plane = { 0 };
		struct v4l2_buffer buffer = {
			.type = BUFFER_TYPE, .memory = V4L2_MEMORY_MMAP,
			.index = i, .length = 1, .m.planes = &plane
		};

		if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
			failure(error, sizeof(error), "VIDIOC_QUERYBUF");
			goto cleanup;
		}
		if (buffer.length != 1 || plane.length < format.fmt.pix_mp.plane_fmt[0].sizeimage) {
			errno = EPROTO;
			failure(error, sizeof(error), "invalid QUERYBUF memory plane");
			goto cleanup;
		}
		maps[i].length = plane.length;
		maps[i].address = mmap(NULL, plane.length, PROT_READ | PROT_WRITE,
				       MAP_SHARED, fd, plane.m.mem_offset);
		if (maps[i].address == MAP_FAILED) {
			maps[i].address = NULL;
			failure(error, sizeof(error), "mmap");
			goto cleanup;
		}
		if (queue(fd, i, maps[i].length) < 0) {
			failure(error, sizeof(error), "initial VIDIOC_QBUF");
			goto cleanup;
		}
	}
	if (interrupted) {
		errno = EINTR;
		failure(error, sizeof(error), "interrupted before STREAMON");
		goto cleanup;
	}
	if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
		failure(error, sizeof(error), "VIDIOC_STREAMON");
		goto cleanup;
	}
	streaming = true;
	while (count < opt.frames) {
		struct v4l2_buffer buffer;
		struct v4l2_plane plane;
		char filename[32];
		size_t length = 0;
		uint64_t ns;

		if (dequeue(fd, &buffer, &plane, opt.timeout_ms) < 0) {
			failure(error, sizeof(error), "poll/VIDIOC_DQBUF");
			goto cleanup;
		}
		if (buffer.index < request.count)
			length = maps[buffer.index].length;
		if (buffer.length != 1 || buffer.field != V4L2_FIELD_NONE ||
		    buffer.index >= request.count ||
		    !length || plane.length != length ||
		    plane.bytesused > length || plane.data_offset > plane.bytesused ||
		    plane.bytesused > format.fmt.pix_mp.plane_fmt[0].sizeimage ||
		    !timestamp_ns(&buffer.timestamp, &ns)) {
			errno = EPROTO;
			failure(error, sizeof(error), "invalid DQBUF bounds/field/timestamp");
			manifest_frame(manifest, records++, NULL, &buffer, &plane, length, error);
			goto cleanup;
		}
		snprintf(filename, sizeof(filename), "frame-%03" PRIu32 ".bin", count);
		if (save_frame(dirfd, filename, maps[buffer.index].address, plane.bytesused) < 0) {
			failure(error, sizeof(error), "save dequeued frame");
			manifest_frame(manifest, records++, filename, &buffer, &plane, length, error);
			goto cleanup;
		}
		manifest_frame(manifest, records++, filename, &buffer, &plane, length, NULL);
		if (fflush(manifest) != 0) {
			failure(error, sizeof(error), "write frame metadata");
			goto cleanup;
		}
		count++;
		if (buffer.flags & V4L2_BUF_FLAG_ERROR)
			error_buffers++;
		/* Keep the final buffer dequeued; STREAMOFF releases the rest. */
		if (count < opt.frames && queue(fd, buffer.index, length) < 0) {
			failure(error, sizeof(error), "requeue VIDIOC_QBUF");
			goto cleanup;
		}
	}
	completed = true;

cleanup:
	if (streaming) {
		if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
			failure(error, sizeof(error), "VIDIOC_STREAMOFF");
			/* Closing the device forces driver teardown before unmapping. */
			close(fd);
			fd = -1;
		}
	}
	for (uint32_t i = 0; i < MAX_BUFFERS; i++) {
		if (maps[i].address && munmap(maps[i].address, maps[i].length) < 0)
			failure(error, sizeof(error), "munmap");
	}
	if (requested && fd >= 0) {
		request.count = 0;
		if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0)
			failure(error, sizeof(error), "release VIDIOC_REQBUFS");
	}
	if (fd >= 0 && close(fd) < 0)
		failure(error, sizeof(error), "close video device");
	completed = completed && !error[0] && !interrupted;
	if (manifest) {
		fprintf(manifest,
			"\n  ],\n  \"capture_complete\":%s, \"exported_frames\":%" PRIu32
			", \"error_buffers\":%" PRIu32 ",\n  \"capture_error\":",
			completed ? "true" : "false", count, error_buffers);
		if (error[0])
			json_string(manifest, error);
		else if (interrupted)
			json_string(manifest, "interrupted by signal");
		else
			fputs("null", manifest);
		fputs(",\n  \"hardware_acceptance\":\"NOT_ESTABLISHED\"\n}\n", manifest);
		if (fflush(manifest) != 0 || fsync(fileno(manifest)) < 0)
			failure(error, sizeof(error), "flush capture.json");
		if (fclose(manifest) != 0)
			failure(error, sizeof(error), "close capture.json");
	}
	if (manifest_fd >= 0)
		close(manifest_fd);
	if (dirfd >= 0 && close(dirfd) < 0)
		failure(error, sizeof(error), "close output directory");
	if (!completed || error[0] || error_buffers) {
		fprintf(stderr, "Capture NOT accepted: exported %" PRIu32 "/%" PRIu32
			", ERROR buffers %" PRIu32 ". Retain all partial evidence.\n",
			count, opt.frames, error_buffers);
		return 1;
	}
	fprintf(stderr, "Exported %" PRIu32 " frames to %s/capture.json."
		" Pixel/color acceptance is still pending.\n", count, opt.output);
	return 0;
}
