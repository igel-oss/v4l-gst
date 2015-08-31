/*
 * Copyright (C) 2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA
 */

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libv4l-plugin.h"

#include "libv4l-gst.h"
#include "gst-backend.h"
#include "evfd-ctrl.h"
#include "debug.h"

#if HAVE_VISIBILITY
#define PLUGIN_PUBLIC __attribute__ ((visibility("default")))
#else
#define PLUGIN_PUBLIC
#endif

static void *plugin_init(int fd)
{
	struct v4l_gst_priv *priv;
	struct stat buf;
	int flags;
	int ret;

	/* Reject character device */
	fstat(fd, &buf);
	if (S_ISCHR(buf.st_mode))
		return NULL;

	DBG_LOG("start plugin_init\n");

	priv = calloc(1, sizeof(*priv));
	if (!priv) {
		perror("Couldn't allocate memory for plugin");
		return NULL;
	}

	flags = fcntl(fd, F_GETFL);
	priv->is_non_blocking = (flags & O_NONBLOCK) ? true : false;
	DBG_LOG("non-blocking : %s\n", (priv->is_non_blocking) ? "on" : "off");

	/*For handling event state */
	priv->event_state = new_event_state();
	if (!priv->event_state)
		goto free_priv;

	if (dup2(priv->event_state->fd, fd) < 0) {
		fprintf(stderr, "dup2 failed\n");
		close(priv->event_state->fd);
		goto free_event;
	}
	close(priv->event_state->fd);

	priv->plugin_fd = fd;
	priv->event_state->fd = fd;

	ret = gst_backend_init(priv);
	if (ret < 0)
		goto free_event;

	DBG_LOG("plugin_init finished\n");

	return priv;
free_event:
	delete_event_state(priv->event_state);

free_priv:
	free(priv);

	return NULL;
}

static void plugin_close(void *dev_ops_priv)
{
	struct v4l_gst_priv *priv = dev_ops_priv;

	if (!priv)
		return;

	gst_backend_deinit(priv);

	delete_event_state(priv->event_state);

	free(dev_ops_priv);
}

static int plugin_ioctl(void *dev_ops_priv, int fd,
			unsigned long int cmd, void *arg)
{
	struct v4l_gst_priv *priv = dev_ops_priv;
	int ret = -1;

	(void)fd; /* unused */

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		ret = querycap_ioctl(priv, arg);
		break;
	case VIDIOC_S_FMT:
		ret = set_fmt_ioctl(priv, arg);
		break;
	case VIDIOC_G_FMT:
		ret = get_fmt_ioctl(priv, arg);
		break;
	case VIDIOC_ENUM_FMT:
		ret = enum_fmt_ioctl(priv, arg);
		break;
	case VIDIOC_G_CTRL:
		ret = get_ctrl_ioctl(priv, arg);
		break;
	case VIDIOC_QBUF:
		ret = qbuf_ioctl(priv, arg);
		break;
	case VIDIOC_DQBUF:
		ret = dqbuf_ioctl(priv, arg);
		break;
	case VIDIOC_QUERYBUF:
		ret = querybuf_ioctl(priv, arg);
		break;
	case VIDIOC_REQBUFS:
		ret = reqbuf_ioctl(priv, arg);
		break;
	case VIDIOC_STREAMON:
		ret = streamon_ioctl(priv, arg);
		break;
	case VIDIOC_STREAMOFF:
		ret = streamoff_ioctl(priv, arg);
		break;
	case VIDIOC_SUBSCRIBE_EVENT:
		ret = subscribe_event_ioctl(priv, arg);
		break;
	case VIDIOC_DQEVENT:
		ret = dqevent_ioctl(priv, arg);
		break;
	default:
		perror("unknown ioctl");
		errno = ENOTTY;
		break;
	}

	return ret;
}

static void *
plugin_mmap(void *dev_ops_priv, void *start, size_t length, int prot,
	    int flags, int fd, int64_t offset)
{
	return gst_backend_mmap(dev_ops_priv, start, length, prot, flags, fd,
				offset);
}

PLUGIN_PUBLIC const struct libv4l_dev_ops libv4l2_plugin = {
	.init = &plugin_init,
	.close = &plugin_close,
	.ioctl = &plugin_ioctl,
	.mmap = &plugin_mmap,
};
