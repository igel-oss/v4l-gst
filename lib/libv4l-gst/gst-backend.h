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

#ifndef __GST_BACKEND_H__
#define __GST_BACKEND_H__

#include <linux/videodev2.h>

#include <gst/video/video.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/allocators/gstdmabuf.h>

#include "libv4l-gst.h"

int gst_backend_init(struct v4l_gst_priv *dev_ops_priv);
void gst_backend_deinit(struct v4l_gst_priv *dev_ops_priv);
int querycap_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_capability *cap);
int set_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt);
int get_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt);
int enum_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_fmtdesc *desc);
int get_ctrl_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_control *ctrl);
int qbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf);
int dqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf);
int querybuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf);
int reqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_requestbuffers *req);
int streamon_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type);
int streamoff_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type);
int subscribe_event_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_event_subscription *sub);
int dqevent_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_event *ev);
int expbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_exportbuffer *buf);
void * gst_backend_mmap(struct v4l_gst_priv *dev_ops_priv, void *start, size_t length, int prot, int flags, int fd, int64_t offset);

#endif /* __GST_BACKEND_H__ */
