#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <pthread.h>

#include <linux/videodev2.h>
#include "libv4l2.h"

#define IN_BUF_NUM 4
#define OUT_BUF_NUM 4

#ifdef DEBUG
#define DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DBG(...)
#endif

int fd;
FILE *in_fp;
int is_eof = 0;
int done = 0;
fpos_t pos;

int event;

const unsigned int in_buf_size = 256*1024;
int in_dequeued[IN_BUF_NUM] = { 0, };

void input_queue(void *arg)
{
	void **in_data = arg;
	struct v4l2_buffer buffer;
	size_t ret;
	struct v4l2_plane planes[1];
	static int tv_cnt = 0;
	int i;

	for (i = 0; i < IN_BUF_NUM; i++) {
		memset(&buffer, 0, sizeof(buffer));
		memset(planes, 0, sizeof(planes));
		buffer.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		buffer.memory   = V4L2_MEMORY_MMAP;
		buffer.m.planes = planes;
		buffer.length   = 1;

		if (v4l2_ioctl(fd, VIDIOC_DQBUF, &buffer) != 0) {
			if (errno == EAGAIN) {
				DBG("output dqbuf EAGAIN\n");
				break;
			}
			fprintf(stderr, "dqbuf failed\n");
			exit(EXIT_FAILURE);
		}
		done = is_eof && buffer.m.planes[0].bytesused == 0;
		in_dequeued[buffer.index] = 1;
	}
	if (is_eof)
		return;

	for (i = 0; i < IN_BUF_NUM; i++) {
		buffer.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		buffer.memory   = V4L2_MEMORY_MMAP;
		buffer.index    = i;
		buffer.m.planes = planes;
		buffer.length   = 1;
		memset(&buffer.timestamp, 0, sizeof(buffer.timestamp));
		buffer.timestamp.tv_sec = tv_cnt;
		tv_cnt++;

		if (!in_dequeued[buffer.index])
			continue;
		ret = fread(in_data[buffer.index], 1, in_buf_size, in_fp);
                fgetpos(in_fp, &pos);
		if (!ret && feof(in_fp))
			is_eof = 1;
		buffer.m.planes[0].bytesused = ret;

		if (v4l2_ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
			fprintf(stderr, "qbuf failed\n");
			exit(EXIT_FAILURE);
		}
		in_dequeued[buffer.index] = 0;
	}
}

void * output_buffer_release(void *data) {
	uint64_t randval = rand();
	int sleep_len = (100 * randval) / (float) RAND_MAX;
	int *wdone = data;
	uint64_t cnt = 1;
	int ret;

	usleep ((int)sleep_len * 1000);
	*wdone = 1;
	ret = write(event, &cnt, 8);
	if (ret == -1) {
		printf ("write error.\n");
		exit(EXIT_FAILURE);
	}

	return NULL;
}

void usage () {
	printf ("usage: liv4l2_test [option] [filename]\n"
		"option list:\n"
		"-h help\n"
		"-b use V4L2 in blocking mode\n");
}

int main(int argc, char *argv[])
{
	int ret;
	struct v4l2_fmtdesc fmtdesc;
	struct v4l2_capability caps;
	struct v4l2_format format;
	const unsigned int kCapsRequired =
			V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			V4L2_CAP_VIDEO_OUTPUT_MPLANE |
			V4L2_CAP_STREAMING;
	struct v4l2_control ctrl;
	struct v4l2_requestbuffers reqbufs;
	struct v4l2_buffer buffer;
	struct v4l2_plane planes[1];
	void *in_data[IN_BUF_NUM];
	void *out_data[OUT_BUF_NUM];
	enum v4l2_buf_type buftype;
	pthread_t thread;
	int dequeued[IN_BUF_NUM] = { 0, };
	FILE *out_fp;
	int i;
	int ch;
	extern char *optarg;
	extern int optind;
	int blocking = 0;
	char *file;

	while ((ch = getopt(argc, argv, "bh")) != -1) {
		switch (ch){
		case 'b':
			blocking = 1;
			break;
		case 'h':
		default:
			usage();
			return 0;
		}
	}
        file = argv[optind];

	if (!file) {
		fprintf (stderr, "Movie file is not specified.\n");
		exit(EXIT_FAILURE);
	}
	if (!blocking)
		fd = open("/dev/video-gst", O_NONBLOCK | O_CLOEXEC);
	else
		fd = open("/dev/video-gst", O_CLOEXEC);

	if (v4l2_fd_open(fd, V4L2_DISABLE_CONVERSION) < 0) {
		fprintf(stderr, "v4l2_fd_open failed\n");
		exit(EXIT_FAILURE);
	}
	event = eventfd(0, 0);

	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	while (v4l2_ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		DBG("output fourcc=0x%08x\n", fmtdesc.pixelformat);
		++fmtdesc.index;
	}

	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	while (v4l2_ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		DBG("capture fourcc=0x%08x\n", fmtdesc.pixelformat);
		++fmtdesc.index;
	}

	if (v4l2_ioctl(fd, VIDIOC_QUERYCAP, &caps) != 0) {
		fprintf(stderr, "querycap failed\n");
		exit(EXIT_FAILURE);
	}

	if ((caps.capabilities & kCapsRequired) == kCapsRequired)
		DBG("kCapsRequired ok\n");
	printf("driver : %s\n", caps.driver);

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	format.fmt.pix_mp.plane_fmt[0].sizeimage = in_buf_size;
	format.fmt.pix_mp.num_planes = 1;
	if (v4l2_ioctl(fd, VIDIOC_S_FMT, &format) == 0)
		printf("VIDIOC_S_FMT V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ok\n");

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_XRGB32;
	if (v4l2_ioctl(fd, VIDIOC_S_FMT, &format) == 0)
		printf("VIDIOC_S_FMT V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ok\n");

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (v4l2_ioctl(fd, VIDIOC_G_FMT, &format) == 0) {
		printf("VIDIOC_G_FMT V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE\n");
		printf("pixelformat=%s\n",
		       (format.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_H264) ?
		       "H264" : "incorrect");
		printf("sizeimage=%d\n", format.fmt.pix_mp.plane_fmt[0].sizeimage);
		printf("num_planes=%d\n", format.fmt.pix_mp.num_planes);
	}
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
	if (v4l2_ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0)
		printf("VIDIOC_G_CTRL: min_buffers=%d\n", ctrl.value);

	memset(&reqbufs, 0, sizeof(reqbufs));
	reqbufs.count  = IN_BUF_NUM;
	reqbufs.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (v4l2_ioctl(fd, VIDIOC_REQBUFS, &reqbufs) == 0)
		printf("VIDIOC_REQBUFS: count = %d\n", reqbufs.count);
	else {
		fprintf(stderr, "VIDIOC_REQBUFS failed\n");
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < IN_BUF_NUM; i++) {
		memset(&buffer, 0, sizeof(buffer));
		memset(planes, 0, sizeof(planes));

		buffer.index    = i;
		buffer.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		buffer.memory   = V4L2_MEMORY_MMAP;
		buffer.m.planes = planes;
		buffer.length   = 1;
		if (v4l2_ioctl(fd, VIDIOC_QUERYBUF, &buffer) == -1) {
			fprintf(stderr, "VIDIOC_QUERYBUF OUTPUT failed index=%d\n", i);
			exit(EXIT_FAILURE);
		}

		DBG("VIDIOC_QUERYBUF index=%d mem_offset=%d\n", i, planes[0].m.mem_offset);

		in_data[i] = v4l2_mmap(NULL, buffer.m.planes[0].length,
				  PROT_READ | PROT_WRITE,
				  MAP_SHARED, fd, buffer.m.planes[0].m.mem_offset);
		if (in_data[i] == MAP_FAILED) {
			fprintf(stderr, "Failed to mmap input buffer (index:%d)\n", i);
			exit(EXIT_FAILURE);
		}

		DBG("OUTPUT plane mapped : %p\n", in_data[i]);
	}

	in_fp = fopen(file, "rb");
	if (!in_fp) {
		fprintf(stderr, "Faile to open test movie\n");
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < IN_BUF_NUM; i++) {
		ret = fread(in_data[i], 1, in_buf_size, in_fp);
		if (ret == -1 ) {
			printf ("read failed\n");
			exit(EXIT_FAILURE);
		}
		memset(&buffer, 0, sizeof(buffer));
		memset(planes, 0, sizeof(planes));

		buffer.index    = i;
		buffer.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		buffer.memory   = V4L2_MEMORY_MMAP;
		buffer.m.planes = planes;
		buffer.m.planes[0].bytesused = in_buf_size;
		buffer.length   = 1;

		if (v4l2_ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
			fprintf(stderr, "qbuf failed\n");
			exit(EXIT_FAILURE);
		}
	}

	buftype = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (v4l2_ioctl(fd, VIDIOC_STREAMON, &buftype) != 0) {
		fprintf(stderr, "output streamon failed\n");
		exit(EXIT_FAILURE);
	}

	while (1) {
		memset(&format, 0, sizeof(format));
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		if (v4l2_ioctl(fd, VIDIOC_G_FMT, &format) == 0) {
			DBG("capture g_fmt success\n");
			break;
		}
		DBG("again capture g_fmt\n");
	}

	printf("g_fmt width=%d height=%d\n",
	       format.fmt.pix_mp.width, format.fmt.pix_mp.height);

	memset(&reqbufs, 0, sizeof(reqbufs));
	reqbufs.count  = OUT_BUF_NUM;
	reqbufs.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (v4l2_ioctl(fd, VIDIOC_REQBUFS, &reqbufs) == 0)
		DBG("VIDIOC_REQBUFS: count = %d\n", reqbufs.count);
	else {
		fprintf(stderr, "VIDIOC_REQBUFS failed\n");
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < OUT_BUF_NUM; i++) {
		struct v4l2_exportbuffer expbuf = {0};
		expbuf.index    = i;
		expbuf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		if (v4l2_ioctl(fd, VIDIOC_EXPBUF, &expbuf) == -1) {
			fprintf(stderr, "VIDIOC_EXPBUF CAPTURE failed index=%d\n", i);
			exit(EXIT_FAILURE);
		}

		DBG("VIDIOC_EXPBUF index=%d fd=%d\n", i, expbuf.fd);
		close(expbuf.fd);
		DBG("CAPTURE plane mapped : %p\n", out_data[i]);
	}

	for (i = 0; i < OUT_BUF_NUM; i++) {
		memset(&buffer, 0, sizeof(buffer));
		memset(planes, 0, sizeof(planes));

		buffer.index    = i;
		buffer.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buffer.memory   = V4L2_MEMORY_MMAP;
		buffer.m.planes = planes;
		buffer.length   = 1;

		DBG("QBUF 000\n");
		if (v4l2_ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
			fprintf(stderr, "qbuf failed\n");
			exit(EXIT_FAILURE);
		}
		DBG("QBUF 001\n");
	}

	DBG("QBUF success\n");

	buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (v4l2_ioctl(fd, VIDIOC_STREAMON, &buftype) != 0) {
		fprintf(stderr, "output streamon failed\n");
		exit(EXIT_FAILURE);
	}


	out_fp = fopen("/dev/null", "wb");
	if (!out_fp) {
		fprintf(stderr, "Faile to open test output file\n");
		exit(EXIT_FAILURE);
	}

	memset(&reqbufs, 0, sizeof(reqbufs));
	reqbufs.count  = OUT_BUF_NUM;
	reqbufs.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (v4l2_ioctl(fd, VIDIOC_REQBUFS, &reqbufs) == 0)
		DBG("VIDIOC_REQBUFS: count = %d\n", reqbufs.count);

	while (!is_eof || !done) {
		pthread_attr_t attr;
		pthread_attr_init(&attr);
		struct pollfd pollfds[2];
		nfds_t nfds;
		int is_buffer_queued = 0;

		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

		for (i = 0; i < IN_BUF_NUM; i++)
			if (!in_dequeued[i])
				is_buffer_queued = 1;

		if (is_buffer_queued) {
			pollfds[0].fd = fd;
			pollfds[0].events = POLLIN | POLLERR;
			pollfds[1].fd = event;
			pollfds[1].events = POLLIN | POLLERR;
			nfds = 2;
			if (poll(pollfds, nfds, -1) == -1) {
				fprintf(stderr, "poll failed\n");
				exit(EXIT_FAILURE);
			}
			if (pollfds[1].revents & POLLIN) {
				char dummy[8];
				ret = read(event, dummy, 8);
				if (ret == -1)
				  exit(EXIT_FAILURE);
			}
		} else {
			DBG("############### no in queued buffers\n");
		}

		input_queue(in_data);

		for (i = 0; i < OUT_BUF_NUM; i++) {
			memset(&buffer, 0, sizeof(buffer));
			memset(planes, 0, sizeof(planes));
			buffer.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buffer.memory   = V4L2_MEMORY_MMAP;
			buffer.m.planes = planes;
			buffer.length   = 1;

			if (v4l2_ioctl(fd, VIDIOC_DQBUF, &buffer) != 0) {
				if (errno == EAGAIN){
					DBG ("capture dqbuf EAGAIN\n");
					break;
				}
				else {
					fprintf(stderr, "dqbuf on CAPTURE failed\n");
					exit(EXIT_FAILURE);
				}
			}
			pthread_create(&thread , &attr, output_buffer_release, &dequeued[buffer.index]);
			DBG("capture dqbuf %d index buffer bytesused=%d\n", buffer.index,
			       buffer.m.planes[0].bytesused);
		}

		for (i = 0; i < OUT_BUF_NUM; i++) {
			memset(&buffer, 0, sizeof(buffer));
			memset(planes, 0, sizeof(planes));

			buffer.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buffer.memory   = V4L2_MEMORY_MMAP;
			buffer.index    = i;
			buffer.m.planes = planes;
			buffer.length   = 1;

			if (!dequeued[buffer.index])
				continue;

			if (v4l2_ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
				fprintf(stderr, "qbuf failed\n");
				exit(EXIT_FAILURE);
			}
			dequeued[buffer.index] = 0;
		}

	}


	buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (v4l2_ioctl(fd, VIDIOC_STREAMOFF, &buftype) != 0) {
		fprintf(stderr, "output streamoff failed\n");
		exit(EXIT_FAILURE);
	}

	buftype = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (v4l2_ioctl(fd, VIDIOC_STREAMOFF, &buftype) != 0) {
		fprintf(stderr, "output streamoff failed\n");
		exit(EXIT_FAILURE);
	}

	printf("v4l2 test success\n");

	v4l2_close(fd);
}


