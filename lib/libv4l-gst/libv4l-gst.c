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

#include <config.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/eventfd.h>

#include <linux/videodev2.h>

#include <gst/video/video.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "libv4l-plugin.h"
#include "libv4l-gst-bufferpool.h"

#if HAVE_VISIBILITY
#define PLUGIN_PUBLIC __attribute__ ((visibility("default")))
#else
#define PLUGIN_PUBLIC
#endif

#define SYS_READ(fd, buf, len) \
	syscall(SYS_read, (int)(fd), (void *)(buf), (size_t)(len));
#define SYS_WRITE(fd, buf, len) \
	syscall(SYS_write, (int)(fd), (const void *)(buf), (size_t)(len));

#ifdef DEBUG
#define DBG_LOG(fmt, ...) \
	fprintf(stderr, "[%s:%d] " fmt, __FUNCTION__, __LINE__, ## __VA_ARGS__)
#else
#define DBG_LOG(fmt, ...)
#endif

#define DEF_CAP_MIN_BUFFERS		2

struct v4l_gst_buffer {
	GstBuffer *buffer;
	GstMapInfo info;
	GstMapFlags flags;

	/* To associate a GstBuffer with a V4L2 buffer in the V4L2 caller side
	   by having the same value as the mem_offset of
	   the v4l2_plane structure. */
	guint buf_id[GST_VIDEO_MAX_PLANES];
};

struct v4l_gst_priv {
	gint plugin_fd;

	GstElement *pipeline;
	GstElement *appsrc;
	GstElement *appsink;

	GstAppSinkCallbacks appsink_cb;

	void *pool_lib_handle;
	struct libv4l_gst_buffer_pool_ops *pool_ops;

	guint *out_fmts;
	gint out_fmts_num;
	guint *cap_fmts;
	gint cap_fmts_num;

	guint out_fourcc;
	gsize out_buf_size;
	struct v4l2_pix_format_mplane cap_pix_fmt;

	gint cap_min_buffers;

	GstBufferPool *src_pool;
	GstBufferPool *sink_pool;

	struct v4l_gst_buffer *out_buffers;
	gint out_buffers_num;
	struct v4l_gst_buffer *cap_buffers;
	gint cap_buffers_num;

	GQueue *reqbufs_queue;

	GQueue *cap_buffers_queue;
	GMutex queue_mutex;
	GCond queue_cond;

	GQueue *out_buffers_queue;

	GThread *out_queue_thread;
	GCond out_queue_cond;

	GstBuffer *pending_buffer;
	GMutex pending_buf_mutex;

	gulong probe_id;

	GMutex cap_reqbuf_mutex;
	GCond cap_reqbuf_cond;

	int is_cap_fmt_acquirable;

	GMutex dev_lock;

	gboolean is_non_blocking;

	gboolean is_cap_fmt_unacquired;
};

struct v4l_gst_format_info {
	guint fourcc;
	GstVideoFormat format;
};

static const gchar* const GST_VIDEO_CODEC_MIME_H264	= "video/x-h264";
static const gchar* const GST_VIDEO_CODEC_MIME_VP8	= "video/x-vp8";

static const struct v4l_gst_format_info v4l_gst_vid_fmt_tbl[] = {
	{ V4L2_PIX_FMT_GREY, GST_VIDEO_FORMAT_GRAY8 },
	{ V4L2_PIX_FMT_RGB565, GST_VIDEO_FORMAT_RGB16 },
	{ V4L2_PIX_FMT_RGB24, GST_VIDEO_FORMAT_RGB },
	{ V4L2_PIX_FMT_BGR24, GST_VIDEO_FORMAT_BGR },
	{ V4L2_PIX_FMT_ABGR32, GST_VIDEO_FORMAT_BGRA },
	{ V4L2_PIX_FMT_XBGR32, GST_VIDEO_FORMAT_BGRx },
	{ V4L2_PIX_FMT_ARGB32, GST_VIDEO_FORMAT_ARGB },
	{ V4L2_PIX_FMT_XRGB32, GST_VIDEO_FORMAT_xRGB },
	{ V4L2_PIX_FMT_NV12, GST_VIDEO_FORMAT_NV12 },
	{ V4L2_PIX_FMT_NV12MT, GST_VIDEO_FORMAT_NV12_64Z32 },
	{ V4L2_PIX_FMT_NV21 ,GST_VIDEO_FORMAT_NV21 },
	{ V4L2_PIX_FMT_NV16, GST_VIDEO_FORMAT_NV16 },
	{ V4L2_PIX_FMT_YVU410, GST_VIDEO_FORMAT_YVU9 },
	{ V4L2_PIX_FMT_YUV410, GST_VIDEO_FORMAT_YUV9 },
	{ V4L2_PIX_FMT_YUV420, GST_VIDEO_FORMAT_I420 },
	{ V4L2_PIX_FMT_YUYV, GST_VIDEO_FORMAT_YUY2 },
	{ V4L2_PIX_FMT_YVU420, GST_VIDEO_FORMAT_YV12 },
	{ V4L2_PIX_FMT_UYVY, GST_VIDEO_FORMAT_UYVY },
	{ V4L2_PIX_FMT_YUV411P, GST_VIDEO_FORMAT_Y41B },
	{ V4L2_PIX_FMT_YUV422P, GST_VIDEO_FORMAT_Y42B },
	{ V4L2_PIX_FMT_YVYU, GST_VIDEO_FORMAT_YVYU },
};

static gboolean already_opened = FALSE;

static gboolean
parse_conf_settings(gchar **pipeline_str, gchar **pool_lib_path,
		    gint *min_buffers)
{
	const gchar *const *sys_conf_dirs;
	GKeyFile *conf_key;
	const gchar *conf_name = "libv4l-gst.conf";
	GError *err = NULL;
	gchar **groups;
	gsize n_groups;
	gboolean ret = FALSE;
	gint i;

	sys_conf_dirs = g_get_system_config_dirs();

	conf_key = g_key_file_new();
	if (!g_key_file_load_from_dirs(conf_key, conf_name,
				       (const gchar **) sys_conf_dirs, NULL,
				       G_KEY_FILE_NONE, &err)) {
		fprintf(stderr, "Failed to load %s "
		       "from the xdg system config directory retrieved from "
		       "XDG_CONFIG_DIRS (%s)\n", conf_name, err->message);
		g_error_free(err);
		goto free_key_file;
	}

	groups = g_key_file_get_groups(conf_key, &n_groups);
	for (i = 0; i < n_groups; i++) {
		if (g_strcmp0(groups[i], "libv4l-gst") != 0)
			/* search next group */
			continue;

		DBG_LOG("libv4l-gst configuration file is found\n");

		err = NULL;
		*pipeline_str = g_key_file_get_string(conf_key, groups[i],
						      "pipeline", &err);
		if (!*pipeline_str) {
			fprintf(stderr, "GStreamer pipeline is not specified\n");
			g_error_free(err);
			goto free_groups;
		}

		DBG_LOG("parsed pipeline : %s\n", *pipeline_str);

		/* No need to check if the external bufferpool library is set,
		   because it is not mandatory for this plugin. */
		*pool_lib_path = g_key_file_get_string(conf_key, groups[i],
						       "bufferpool-library",
						       NULL);

		DBG_LOG("external buffer pool library : %s\n",
			*pool_lib_path ? *pool_lib_path : "none");

		*min_buffers = g_key_file_get_integer(conf_key, groups[i],
						      "min-buffers", NULL);
		if (*min_buffers == 0)
			*min_buffers = DEF_CAP_MIN_BUFFERS;

		DBG_LOG("minimum number of buffers on CAPTURE "
			"for the GStreamer pipeline to work : %d\n",
			*min_buffers);

		break;
	}

	ret = TRUE;

free_groups:
	g_strfreev(groups);
free_key_file:
	g_key_file_free(conf_key);

	return ret;
}

static GstElement *
create_pipeline(gchar *pipeline_str)
{
	gchar *launch_str;
	GstElement *pipeline;
	GError *err = NULL;

	gst_init(NULL, NULL);

	launch_str = g_strdup_printf("appsrc ! %s ! appsink sync=false",
				     pipeline_str);

	DBG_LOG("gst_parse_launch: %s\n", launch_str);

	pipeline = gst_parse_launch(launch_str, &err);
	g_free(launch_str);

	if (err) {
		fprintf(stderr, "Couldn't construct pipeline: %s\n",
			err->message);
		g_error_free(err);
		return NULL;
	}

	return pipeline;
}

static gboolean
get_app_elements(GstElement *pipeline, GstElement **appsrc,
		 GstElement **appsink)
{
	GstIterator *it;
	gboolean done = FALSE;
	GValue data = { 0, };
	GstElement *elem;
	GstElement *src_elem, *sink_elem;
	GstElementFactory *factory;
	const gchar *elem_name;

	src_elem = sink_elem = NULL;

	it = gst_bin_iterate_elements(GST_BIN(pipeline));
	while (!done) {
		switch (gst_iterator_next(it, &data)) {
		case GST_ITERATOR_OK:
			elem = g_value_get_object(&data);

			factory = gst_element_get_factory(elem);
			elem_name =
				gst_element_factory_get_metadata(factory,
						GST_ELEMENT_METADATA_LONGNAME);
			if (g_strcmp0(elem_name, "AppSrc") == 0)
				src_elem = elem;
			else if (g_strcmp0(elem_name, "AppSink") == 0)
				sink_elem = elem;

			g_value_reset(&data);
			break;
		case GST_ITERATOR_DONE:
		default:
			done = TRUE;
			break;
		}
	}

	g_value_unset(&data);
	gst_iterator_free(it);

	if (!src_elem || !sink_elem) {
		fprintf(stderr,
			"Failed to get app elements from the pipeline\n");
		return FALSE;
	}

	*appsrc = src_elem;
	*appsink = sink_elem;

	DBG_LOG("appsrc and appsink elements are found in the pipeline\n");

	return TRUE;
}

static void
get_buffer_pool_ops(gchar *pool_lib_path, void **pool_lib_handle,
		    struct libv4l_gst_buffer_pool_ops **pool_ops)
{
	void *handle;
	gchar *err;
	struct libv4l_gst_buffer_pool_ops *ops;

	handle = dlopen(pool_lib_path, RTLD_LAZY);
	if (!handle) {
		fprintf(stderr, "dlopen failed (%s)\n", dlerror());
		return;
	}

	dlerror(); /* Clear any existing error */

	ops = dlsym(handle, "libv4l_gst_bufferpool");
	err = dlerror();
	if (err) {
		fprintf(stderr, "dlsym failed (%s)\n", err);
		dlclose(handle);
		return;
	}

	*pool_lib_handle = handle;
	*pool_ops = ops;

	DBG_LOG("buffer pool ops is set\n");
}

static guint
convert_video_format_gst_to_v4l2(GstVideoFormat fmt)
{
	gint i;
	guint fourcc = 0;

	for (i = 0; i < G_N_ELEMENTS(v4l_gst_vid_fmt_tbl); i++)
		if (v4l_gst_vid_fmt_tbl[i].format == fmt)
			fourcc = v4l_gst_vid_fmt_tbl[i].fourcc;

	return fourcc;
}

static GstVideoFormat
convert_video_format_v4l2_to_gst(guint fourcc)
{
	gint i;
	GstVideoFormat fmt = GST_VIDEO_FORMAT_UNKNOWN;

	for (i = 0; i < G_N_ELEMENTS(v4l_gst_vid_fmt_tbl); i++)
		if (v4l_gst_vid_fmt_tbl[i].fourcc == fourcc)
			fmt = v4l_gst_vid_fmt_tbl[i].format;

	return fmt;
}

static const gchar *
convert_codec_type_v4l2_to_gst(guint fourcc)
{
	const gchar *mime;

	if (fourcc == V4L2_PIX_FMT_H264)
		mime = GST_VIDEO_CODEC_MIME_H264;
	else if (fourcc == V4L2_PIX_FMT_VP8)
		mime = GST_VIDEO_CODEC_MIME_VP8;
	else
		mime = NULL;

	return mime;
}

static GstPad *
get_peer_pad(GstElement *elem, const gchar *pad_name)
{
	GstPad *pad;
	GstPad *peer_pad;

	pad = gst_element_get_static_pad(elem, pad_name);
	peer_pad = gst_pad_get_peer(pad);
	gst_object_unref(pad);

	return peer_pad;
}

static GstElement *
get_peer_element(GstElement *elem, const gchar *pad_name)
{
	GstPad *peer_pad;
	GstElement *peer_elem;

	peer_pad = get_peer_pad(elem, pad_name);
	peer_elem = gst_pad_get_parent_element(peer_pad);
	gst_object_unref(peer_pad);

	return peer_elem;
}

static GstCaps *
get_peer_pad_template_caps(GstElement *elem, const gchar *pad_name)
{
	GstPad *peer_pad;
	GstCaps *caps;

	peer_pad = get_peer_pad(elem, pad_name);
	caps = GST_PAD_TEMPLATE_CAPS(GST_PAD_PAD_TEMPLATE(peer_pad));
	gst_caps_ref(caps);
	gst_object_unref(peer_pad);

	return caps;
}

static gboolean
get_supported_video_format_out(GstElement *appsrc, guint **out_fmts,
			       gint *out_fmts_num)
{
	GstCaps *caps;
	GstStructure *structure;
	const gchar *mime;
	guint fourcc;

	caps = get_peer_pad_template_caps(appsrc, "src");

	if (gst_caps_is_any(caps)) {
		/* H.264 and VP8 codecs are supported in this plugin.
		   We treat all the codecs when GST_CAPS_ANY is set as
		   a template caps. */
		*out_fmts_num = 2;
		*out_fmts = g_new(guint, *out_fmts_num);

		*out_fmts[0] = V4L2_PIX_FMT_H264;
		*out_fmts[1] = V4L2_PIX_FMT_VP8;

		DBG_LOG("out supported codecs : h264, vp8\n");
	} else {
		structure = gst_caps_get_structure(caps, 0);
		mime = gst_structure_get_name(structure);

		if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_H264) == 0)
			fourcc = V4L2_PIX_FMT_H264;
		else if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_VP8) == 0)
			fourcc = V4L2_PIX_FMT_VP8;
		else {
			fprintf(stderr, "Unsupported codec : %s\n", mime);
			gst_caps_unref(caps);
			return FALSE;
		}

		DBG_LOG("out supported codec : %s\n", mime);

		*out_fmts_num = 1;
		*out_fmts = g_new(guint, *out_fmts_num);
		*out_fmts[0] = fourcc;
	}

	gst_caps_unref(caps);

	return TRUE;
}

static gboolean
get_supported_video_format_cap(GstElement *appsink, guint **cap_fmts,
			       gint *cap_fmts_num)
{
	GstCaps *caps;
	GstStructure *structure;
	const GValue *val, *list_val;
	const gchar *fmt_str;
	GstVideoFormat fmt;
	guint fourcc;
	gint list_size;
	gint fmts_num;
	guint i;

	caps = get_peer_pad_template_caps(appsink, "sink");

	/* We treat GST_CAPS_ANY as all video formats support. */
	if (gst_caps_is_any(caps)) {
		gst_caps_unref(caps);
		caps = gst_caps_from_string
				("video/x-raw, format=" GST_VIDEO_FORMATS_ALL);
	}

	structure = gst_caps_get_structure(caps, 0);
	val = gst_structure_get_value(structure, "format");
	if (!val) {
		fprintf(stderr, "Failed to get video formats from caps\n");
		gst_caps_unref(caps);
		return FALSE;
	}

	list_size = gst_value_list_get_size(val);
	*cap_fmts = g_new(guint, list_size);

	for (i = 0, fmts_num = 0; i < list_size; i++) {
		list_val = gst_value_list_get_value(val, i);
		fmt_str = g_value_get_string(list_val);

		fmt = gst_video_format_from_string(fmt_str);
		if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
			fprintf(stderr, "Unknown video format : %s\n", fmt_str);
			continue;
		}

		fourcc = convert_video_format_gst_to_v4l2(fmt);
		if (fourcc == 0) {
			DBG_LOG("Failed to convert video format "
				"from gst to v4l2 : %s\n", fmt_str);
			continue;
		}

		DBG_LOG("cap supported video format : %s\n", fmt_str);

		(*cap_fmts)[fmts_num++] = fourcc;
	}

	*cap_fmts_num = fmts_num;

	DBG_LOG("The total number of cap supported video format : %d\n",
		*cap_fmts_num);

	gst_caps_unref(caps);

	return TRUE;
}

static void
create_buffer_pool(struct libv4l_gst_buffer_pool_ops *pool_ops,
		   GstBufferPool **src_pool, GstBufferPool **sink_pool)
{
	if (pool_ops) {
		if (pool_ops->add_external_src_buffer_pool)
			*src_pool = pool_ops->add_external_src_buffer_pool();

		if (pool_ops->add_external_sink_buffer_pool)
			*sink_pool = pool_ops->add_external_sink_buffer_pool();
	}

	/* fallback to the default buffer pool */
	if (!*src_pool)
		*src_pool = gst_buffer_pool_new();
	if (!*sink_pool)
		*sink_pool = gst_buffer_pool_new();
}

static void
set_buffer_pool_params(GstBufferPool *pool, GstCaps *caps, guint buf_size,
		       guint min_buffers, guint max_buffers)
{
	GstStructure *config;

	config = gst_buffer_pool_get_config(pool);
	gst_buffer_pool_config_set_params(config, caps, buf_size, min_buffers,
					  max_buffers);
	gst_buffer_pool_set_config(pool, config);
}

static void
get_buffer_pool_params(GstBufferPool *pool, GstCaps **caps, guint *buf_size,
		       guint *min_buffers, guint *max_buffers)
{
	GstStructure *config;

	config = gst_buffer_pool_get_config(pool);
	gst_buffer_pool_config_get_params(config, caps, buf_size, min_buffers,
					  max_buffers);
	gst_structure_free(config);
}

static void
retrieve_cap_format_info(struct v4l_gst_priv *dev_ops_priv, GstVideoInfo *info)
{
	gint fourcc;

	dev_ops_priv->cap_pix_fmt.width = info->width;
	dev_ops_priv->cap_pix_fmt.height = info->height;

	fourcc = convert_video_format_gst_to_v4l2(info->finfo->format);
	if (dev_ops_priv->cap_pix_fmt.pixelformat != 0 &&
	    dev_ops_priv->cap_pix_fmt.pixelformat != fourcc)
		fprintf(stderr, "WARNING: Unexpected cap video format\n");
	dev_ops_priv->cap_pix_fmt.pixelformat = fourcc;

	dev_ops_priv->cap_pix_fmt.num_planes = info->finfo->n_planes;
}

static void
wait_for_cap_reqbuf_invocation(struct v4l_gst_priv *dev_ops_priv)
{
	g_mutex_lock(&dev_ops_priv->cap_reqbuf_mutex);
	while (dev_ops_priv->cap_buffers_num <= 0)
		g_cond_wait(&dev_ops_priv->cap_reqbuf_cond,
			    &dev_ops_priv->cap_reqbuf_mutex);
	g_mutex_unlock(&dev_ops_priv->cap_reqbuf_mutex);
}

static GstPadProbeReturn
pad_probe_query(GstPad *pad, GstPadProbeInfo *probe_info, gpointer user_data)
{
	struct v4l_gst_priv *dev_ops_priv = user_data;
	GstQuery *query;
	GstCaps *caps;
	GstVideoInfo info;
	guint64 buf = 1;
	GstBuffer *buffer = NULL;

	query = GST_PAD_PROBE_INFO_QUERY (probe_info);
	if (GST_QUERY_TYPE (query) == GST_QUERY_ALLOCATION &&
	    GST_PAD_PROBE_INFO_TYPE (probe_info) & GST_PAD_PROBE_TYPE_PUSH) {
		DBG_LOG("parse allocation query\n");
		gst_query_parse_allocation(query, &caps, NULL);
		if (!caps) {
			fprintf(stderr, "No caps in query\n");
			return GST_PAD_PROBE_OK;
		}

		if (!gst_video_info_from_caps(&info, caps)) {
			fprintf(stderr, "Failed to get video info\n");
			return GST_PAD_PROBE_OK;
		}

		/* Workaround: gst-omx arouses caps negotiations toward
		   downstream twice.
		   The first of them always has the QCIF resolution
		   and we skip it to receive the only second query that
		   has the actual video parameters. */
		if (info.width == 176 && info.height == 144)
			return GST_PAD_PROBE_OK;

		retrieve_cap_format_info(dev_ops_priv, &info);

		g_mutex_lock(&dev_ops_priv->pending_buf_mutex);
		/* Enable the video parameters acquisition
		   by VIDIOC_G_FMT ioctl on CAPTURE */
		g_atomic_int_set(&dev_ops_priv->is_cap_fmt_acquirable, 1);
		if (dev_ops_priv->pending_buffer) {
			buffer = dev_ops_priv->pending_buffer;
			dev_ops_priv->pending_buffer = NULL;
		}
		g_mutex_unlock(&dev_ops_priv->pending_buf_mutex);

		/* As the initial decoding processing, the application can
		   continue to queue an input stream until the capture format
		   has been acquirable. In this case, the buffers queued on
		   the output type could not be consumed and stay queued
		   in the GStreame pipeline because the buffers on
		   the capture type are not prepared yet. That makes
		   the application wait at the poll forever when using
		   the non-blocking mode.
		   To avoid this race condition, the plugin holds one of
		   the buffers on the output type and will release it
		   right after the capture format has been acquirable, emitting
		   POLLIN. */
		g_mutex_lock(&dev_ops_priv->queue_mutex);
		if (buffer)
			g_queue_push_tail(dev_ops_priv->out_buffers_queue,
					  buffer);
		DBG_LOG("Emit POLLIN\n");
		SYS_WRITE(dev_ops_priv->plugin_fd, &buf, sizeof(buf));
		dev_ops_priv->is_cap_fmt_unacquired = TRUE;
		g_mutex_unlock(&dev_ops_priv->queue_mutex);

		wait_for_cap_reqbuf_invocation(dev_ops_priv);

		set_buffer_pool_params(dev_ops_priv->sink_pool, caps, info.size,
				       dev_ops_priv->cap_buffers_num,
				       dev_ops_priv->cap_buffers_num);

		gst_query_add_allocation_pool(query, dev_ops_priv->sink_pool,
					      info.size,
					      dev_ops_priv->cap_buffers_num,
					      dev_ops_priv->cap_buffers_num);
		gst_object_unref(dev_ops_priv->sink_pool);
	}

	return GST_PAD_PROBE_OK;
}

static gulong
setup_query_pad_probe(struct v4l_gst_priv *dev_ops_priv)
{
	GstPad *peer_pad;
	gulong probe_id;

	peer_pad = get_peer_pad(dev_ops_priv->appsink, "sink");
	probe_id = gst_pad_add_probe(peer_pad,
				     GST_PAD_PROBE_TYPE_QUERY_DOWNSTREAM,
				     (GstPadProbeCallback) pad_probe_query,
				     dev_ops_priv, NULL);
	gst_object_unref(peer_pad);

	return probe_id;
}

static GstBuffer *
pull_buffer_from_sample(GstAppSink *appsink)
{
	GstSample *sample;
	GstBuffer *buffer;

	sample = gst_app_sink_pull_sample(appsink);
	buffer = gst_sample_get_buffer(sample);
	gst_buffer_ref(buffer);
	gst_sample_unref(sample);

	return buffer;
}

static GstFlowReturn
appsink_callback_new_sample(GstAppSink *appsink, gpointer user_data)
{
	struct v4l_gst_priv *dev_ops_priv = user_data;
	GstBuffer *buffer;
	gboolean is_empty;
	GQueue *queue;
	guint64 buf = 1;

	buffer = pull_buffer_from_sample(appsink);

	DBG_LOG("pull buffer: %p\n", buffer);

	if (dev_ops_priv->cap_buffers)
		queue = dev_ops_priv->cap_buffers_queue;
	else
		queue = dev_ops_priv->reqbufs_queue;

	g_mutex_lock(&dev_ops_priv->queue_mutex);

	is_empty = g_queue_is_empty(queue);
	g_queue_push_tail(queue, buffer);

	if (is_empty) {
		g_cond_signal(&dev_ops_priv->queue_cond);
		DBG_LOG("Emit POLLIN\n");
		SYS_WRITE(dev_ops_priv->plugin_fd, &buf, sizeof(buf));
	} else if (!dev_ops_priv->cap_buffers)
		g_cond_signal(&dev_ops_priv->queue_cond);

	g_mutex_unlock(&dev_ops_priv->queue_mutex);

	return GST_FLOW_OK;
}

static gboolean
init_app_elements(struct v4l_gst_priv *dev_ops_priv)
{
	guint *out_fmts;
	gint out_fmts_num;
	guint *cap_fmts;
	gint cap_fmts_num;

	/* Get appsrc and appsink elements respectively from the pipeline */
	if (!get_app_elements(dev_ops_priv->pipeline, &dev_ops_priv->appsrc,
			      &dev_ops_priv->appsink))
		return FALSE;

	/* Set the appsrc queue size to unlimited.
	   The amount of buffers is managed by the buffer pool. */
	gst_app_src_set_max_bytes(GST_APP_SRC(dev_ops_priv->appsrc), 0);

	dev_ops_priv->appsink_cb.new_sample = appsink_callback_new_sample;
	gst_app_sink_set_callbacks(GST_APP_SINK(dev_ops_priv->appsink),
				   &dev_ops_priv->appsink_cb, dev_ops_priv,
				   NULL);

	/* For queuing buffers received from appsink */
	dev_ops_priv->cap_buffers_queue = g_queue_new();
	dev_ops_priv->reqbufs_queue = g_queue_new();
	g_mutex_init(&dev_ops_priv->queue_mutex);
	g_cond_init(&dev_ops_priv->queue_cond);

	/* For queuing buffers exhausted by appsrc */
	dev_ops_priv->out_buffers_queue = g_queue_new();
	g_cond_init(&dev_ops_priv->out_queue_cond);

	g_mutex_init(&dev_ops_priv->pending_buf_mutex);

	if (!get_supported_video_format_out(dev_ops_priv->appsrc, &out_fmts,
					    &out_fmts_num))
		return FALSE;
	if (!get_supported_video_format_cap(dev_ops_priv->appsink, &cap_fmts,
					    &cap_fmts_num)) {
		g_free(out_fmts);
		return FALSE;
	}

	dev_ops_priv->out_fmts = out_fmts;
	dev_ops_priv->out_fmts_num = out_fmts_num;
	dev_ops_priv->cap_fmts = cap_fmts;
	dev_ops_priv->cap_fmts_num = cap_fmts_num;

	return TRUE;
}

static gboolean
init_buffer_pool(struct v4l_gst_priv *dev_ops_priv, gchar *pool_lib_path)
{
	/* Get the external buffer pool when it is specified in
	   the configuration file */
	if (pool_lib_path)
		get_buffer_pool_ops(pool_lib_path,
				    &dev_ops_priv->pool_lib_handle,
				    &dev_ops_priv->pool_ops);

	create_buffer_pool(dev_ops_priv->pool_ops, &dev_ops_priv->src_pool,
			   &dev_ops_priv->sink_pool);

	/* To hook allocation queries */
	dev_ops_priv->probe_id = setup_query_pad_probe(dev_ops_priv);
	if (dev_ops_priv->probe_id == 0) {
		fprintf(stderr, "Failed to setup query pad probe\n");
		goto free_pool;
	}

	/* To wait for the requested number of buffers on CAPTURE
	   to be set in pad_probe_query() */
	g_mutex_init(&dev_ops_priv->cap_reqbuf_mutex);
	g_cond_init(&dev_ops_priv->cap_reqbuf_cond);

	return TRUE;

	/* error cases */
free_pool:
	gst_object_unref(dev_ops_priv->src_pool);
	gst_object_unref(dev_ops_priv->sink_pool);

	return FALSE;
}

static void *plugin_init(int fd)
{
	struct v4l_gst_priv *priv;
	gchar *pipeline_str = NULL;
	gchar *pool_lib_path = NULL;
	gint flags;
	int efd;

	/* Forbid multiple open */
	if (already_opened)
		return NULL;

	DBG_LOG("start plugin_init\n");

	priv = calloc(1, sizeof(*priv));
	if (!priv) {
		perror("Couldn't allocate memory for plugin");
		return NULL;
	}

	flags = fcntl(fd, F_GETFL);
	priv->is_non_blocking = (flags & O_NONBLOCK) ? TRUE : FALSE;
	DBG_LOG("non-blocking : %s\n", (priv->is_non_blocking) ? "on" : "off");

	efd = eventfd(0, EFD_NONBLOCK);
	if (efd < 0) {
		fprintf(stderr, "eventfd failed\n");
		goto free_priv;
	}

	if (dup2(efd, fd) < 0) {
		fprintf(stderr, "dup2 failed\n");
		goto free_priv;
	}
	close(efd);

	priv->plugin_fd = fd;

	if (!parse_conf_settings(&pipeline_str, &pool_lib_path,
				 &priv->cap_min_buffers))
		goto free_priv;

	priv->pipeline = create_pipeline(pipeline_str);
	g_free(pipeline_str);

	if (!priv->pipeline)
		goto free_pool_path;

	/* Initialization regarding appsrc and appsink elements */
	if (!init_app_elements(priv))
		goto free_pipeline;

	if (!init_buffer_pool(priv, pool_lib_path))
		goto free_app_elems_init_objs;
	g_free(pool_lib_path);

	g_mutex_init(&priv->dev_lock);

	already_opened = TRUE;

	DBG_LOG("plugin_init finished\n");

	return priv;

	/* error cases */
free_app_elems_init_objs:
	g_queue_free(priv->cap_buffers_queue);
	g_queue_free(priv->reqbufs_queue);
	g_mutex_clear(&priv->queue_mutex);
	g_cond_clear(&priv->queue_cond);
	g_cond_clear(&priv->out_queue_cond);
	g_mutex_clear(&priv->pending_buf_mutex);
	g_free(priv->out_fmts);
	g_free(priv->cap_fmts);
free_pipeline:
	gst_object_unref(priv->pipeline);
free_pool_path:
	g_free(pool_lib_path);
free_priv:
	free(priv);

	return NULL;
}

static void
remove_query_pad_probe(GstElement *appsink, gulong probe_id)
{
	GstPad *peer_pad;

	peer_pad = get_peer_pad(appsink, "sink");
	gst_pad_remove_probe(peer_pad, probe_id);
	gst_object_unref(peer_pad);
}

static void plugin_close(void *dev_ops_priv)
{
	struct v4l_gst_priv *priv = dev_ops_priv;

	if (!priv)
		return;

	g_mutex_clear(&priv->dev_lock);

	remove_query_pad_probe(priv->appsink, priv->probe_id);

	if (priv->out_buffers)
		g_free(priv->out_buffers);

	if (priv->cap_buffers)
		g_free(priv->cap_buffers);

	gst_object_unref(priv->src_pool);
	gst_object_unref(priv->sink_pool);

	if (priv->pool_lib_handle)
		dlclose(priv->pool_lib_handle);

	g_free(priv->out_fmts);
	g_free(priv->cap_fmts);

	g_queue_free(priv->cap_buffers_queue);
	g_queue_free(priv->reqbufs_queue);
	g_mutex_clear(&priv->queue_mutex);
	g_cond_clear(&priv->queue_cond);

	g_mutex_clear(&priv->pending_buf_mutex);

	g_mutex_clear(&priv->cap_reqbuf_mutex);
	g_cond_clear(&priv->cap_reqbuf_cond);

	gst_object_unref(priv->pipeline);

	free(dev_ops_priv);

	already_opened = FALSE;
}

static int
querycap_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_capability *cap)
{
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE
			  | V4L2_CAP_VIDEO_OUTPUT_MPLANE
			  | V4L2_CAP_STREAMING;

	g_strlcpy((gchar *)cap->driver, "libv4l-gst", sizeof(cap->driver));

	return 0;
}

static gboolean
is_pix_fmt_supported(guint *fmts, gint fmts_num, guint fourcc)
{
	gint i;
	gboolean ret = FALSE;

	for (i = 0; i < fmts_num; i++)
		if (fmts[i] == fourcc) {
			ret = TRUE;
			break;
		}

	return ret;
}

static void
set_params_as_encoded_stream(struct v4l2_pix_format_mplane *pix_fmt)
{
	/* We set the following parameters assuming that encoded streams are
	   received on the output buffer type. The values are almost
	   meaningless. */
	pix_fmt->width = 0;
	pix_fmt->height = 0;
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = 0;
	pix_fmt->flags = 0;
	pix_fmt->plane_fmt[0].bytesperline = 0;
	pix_fmt->num_planes = 1;
}

static int
set_fmt_ioctl_out(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt;

	pix_fmt = &fmt->fmt.pix_mp;

	if (!is_pix_fmt_supported(dev_ops_priv->out_fmts,
				  dev_ops_priv->out_fmts_num,
				  pix_fmt->pixelformat)) {
		fprintf(stderr, "Unsupported pixelformat on OUTPUT\n");
		errno = EINVAL;
		return -1;
	}

	if (pix_fmt->plane_fmt[0].sizeimage == 0) {
		fprintf(stderr, "sizeimage field is not specified on OUTPUT\n");
		errno = EINVAL;
		return -1;
	}

	dev_ops_priv->out_fourcc = pix_fmt->pixelformat;
	dev_ops_priv->out_buf_size = pix_fmt->plane_fmt[0].sizeimage;

	set_params_as_encoded_stream(pix_fmt);

	return 0;
}

static void
init_decoded_frame_params(struct v4l2_pix_format_mplane *pix_fmt)
{
	/* The following parameters will be determined after
	   the video decoding starts. */
	pix_fmt->width = 0;
	pix_fmt->height = 0;
	pix_fmt->num_planes = 0;
	memset(pix_fmt->plane_fmt, 0, sizeof(pix_fmt->plane_fmt));
}

static int
set_fmt_ioctl_cap(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt;

	pix_fmt = &fmt->fmt.pix_mp;

	if (!is_pix_fmt_supported(dev_ops_priv->cap_fmts,
				  dev_ops_priv->cap_fmts_num,
				  pix_fmt->pixelformat)) {
		fprintf(stderr, "Unsupported pixelformat on CAPTURE\n");
		errno = EINVAL;
		return -1;
	}

	dev_ops_priv->cap_pix_fmt.pixelformat = pix_fmt->pixelformat;

	init_decoded_frame_params(pix_fmt);

	/* set unsupported parameters */
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = 0;
	pix_fmt->flags = 0;

	return 0;
}

static int
set_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	int ret;

	if (GST_STATE(dev_ops_priv->pipeline) != GST_STATE_NULL) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		return -1;
	}

	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = set_fmt_ioctl_out(dev_ops_priv, fmt);
	else if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = set_fmt_ioctl_cap(dev_ops_priv, fmt);
	else {
		fprintf(stderr, "Invalid buffer type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

static int
get_fmt_ioctl_cap(struct v4l_gst_priv *dev_ops_priv,
		  struct v4l2_pix_format_mplane *pix_fmt)
{
	gint i;
	guint64 buf;

	if (!g_atomic_int_get(&dev_ops_priv->is_cap_fmt_acquirable)) {
		errno = EINVAL;
		return -1;
	}

	DBG_LOG("cap format is acquirable\n");

	pix_fmt->width = dev_ops_priv->cap_pix_fmt.width;
	pix_fmt->height = dev_ops_priv->cap_pix_fmt.height;
	pix_fmt->pixelformat = dev_ops_priv->cap_pix_fmt.pixelformat;
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = 0;
	pix_fmt->flags = 0;
	pix_fmt->num_planes = dev_ops_priv->cap_pix_fmt.num_planes;

	DBG_LOG("width:%d height:%d num_plnaes=%d\n",
		pix_fmt->width, pix_fmt->height, pix_fmt->num_planes);

	if (dev_ops_priv->cap_pix_fmt.plane_fmt[0].sizeimage > 0) {
		for (i = 0; i < pix_fmt->num_planes; i++) {
			pix_fmt->plane_fmt[i].sizeimage =
					dev_ops_priv->
					cap_pix_fmt.plane_fmt[i].sizeimage;
			pix_fmt->plane_fmt[i].bytesperline =
					dev_ops_priv->
					cap_pix_fmt.plane_fmt[i].bytesperline;
		}
		pix_fmt->num_planes = dev_ops_priv->cap_pix_fmt.num_planes;
	} else
		memset(pix_fmt->plane_fmt, 0, sizeof(pix_fmt->plane_fmt));

	g_mutex_lock(&dev_ops_priv->queue_mutex);
	if (g_queue_is_empty(dev_ops_priv->out_buffers_queue) &&
	    g_queue_is_empty(dev_ops_priv->cap_buffers_queue)) {
		DBG_LOG("Remove POLLIN\n");
		SYS_READ(dev_ops_priv->plugin_fd, &buf, sizeof(buf));
	}
	dev_ops_priv->is_cap_fmt_unacquired = FALSE;
	g_mutex_unlock(&dev_ops_priv->queue_mutex);

	return 0;
}

static int
get_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt;
	int ret;

	pix_fmt = &fmt->fmt.pix_mp;

	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_fmt->pixelformat = dev_ops_priv->out_fourcc;
		pix_fmt->plane_fmt[0].sizeimage = dev_ops_priv->out_buf_size;
		set_params_as_encoded_stream(pix_fmt);
		ret = 0;
	} else if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = get_fmt_ioctl_cap(dev_ops_priv, pix_fmt);
	else {
		fprintf(stderr, "Invalid buffer type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

static int
enum_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_fmtdesc *desc)
{
	guint *fmts;
	gint fmts_num;

	if (!dev_ops_priv->out_fmts || !dev_ops_priv->cap_fmts) {
		fprintf(stderr, "Supported formats lists are not prepared\n");
		errno = EINVAL;
		return -1;
	}

	if (desc->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmts = dev_ops_priv->out_fmts;
		fmts_num = dev_ops_priv->out_fmts_num;
	} else if (desc->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmts = dev_ops_priv->cap_fmts;
		fmts_num = dev_ops_priv->cap_fmts_num;
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		return -1;
	}

	if (fmts_num <= desc->index) {
		errno = EINVAL;
		return -1;
	}

	desc->pixelformat = fmts[desc->index];

	return 0;
}

static int
get_ctrl_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_control *ctrl)
{
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->value = dev_ops_priv->cap_min_buffers;
		ret = 0;
		break;
	default:
		fprintf(stderr, "Invalid control id\n");
		errno = EINVAL;
		ret = -1;
		break;
	}

	return ret;
}

static gboolean
is_supported_memory_io(enum v4l2_memory memory)
{
	if (memory != V4L2_MEMORY_MMAP) {
		errno = EINVAL;
		return FALSE;
	}

	return TRUE;
}

static gboolean
get_raw_video_params(GstBufferPool *pool, GstBuffer *buffer, GstVideoInfo *info,
		     GstVideoMeta **meta)
{
	gboolean ret;
	GstCaps *caps;
	GstVideoInfo vinfo;
	GstVideoMeta *vmeta;

	get_buffer_pool_params(pool, &caps, NULL, NULL, NULL);

	ret = gst_video_info_from_caps(&vinfo, caps);
	if (!ret || GST_VIDEO_INFO_FORMAT(&vinfo) == GST_VIDEO_FORMAT_ENCODED)
		return FALSE;

	vmeta = gst_buffer_get_video_meta(buffer);
	if (!vmeta)
		return FALSE;

	if (info)
		memcpy(info, &vinfo, sizeof(GstVideoInfo));
	if (meta)
		*meta = vmeta;

	return TRUE;
}

/* This check passes through the verification of the buffer index. */
static gboolean
check_no_index_v4l2_buffer(struct v4l2_buffer *buf,
			   struct v4l_gst_buffer *buffers, GstBufferPool *pool)
{
	GstVideoMeta *meta;
	guint n_planes;

	if (!is_supported_memory_io(buf->memory))
		return FALSE;

	if (!buf->m.planes) {
		fprintf(stderr, "This plugin supports only multi-planar "
			"buffer type, but planes array is not set\n");
		errno = EINVAL;
		return FALSE;
	}

	if (get_raw_video_params(pool, buffers[buf->index].buffer, NULL,
				 &meta))
		n_planes = meta->n_planes;
	else
		n_planes = 1;

	if (buf->length < n_planes || buf->length > VIDEO_MAX_PLANES) {
		fprintf(stderr, "Incorrect planes array length\n");
		errno = EINVAL;
		return FALSE;
	}

	return TRUE;
}

static gboolean
check_v4l2_buffer(struct v4l2_buffer *buf, struct v4l_gst_buffer *buffers,
		  gint buffers_num, GstBufferPool *pool)
{
	if (!check_no_index_v4l2_buffer(buf, buffers, pool))
		return FALSE;

	if (buf->index >= buffers_num) {
		fprintf(stderr, "buffer index is out of range\n");
		errno = EINVAL;
		return FALSE;
	}

	return TRUE;
}

static void
notify_unref(gpointer data)
{
	GstBuffer *buffer = data;

	DBG_LOG("unref buffer: %p\n", buffer);
	gst_buffer_unref(buffer);
}

static int
qbuf_ioctl_out(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	GstFlowReturn flow_ret;
	GstBuffer *buffer;
	GstMapInfo info;

	if (!check_v4l2_buffer(buf, dev_ops_priv->out_buffers,
			       dev_ops_priv->out_buffers_num,
			       dev_ops_priv->src_pool))
		return -1;

	DBG_LOG("queue index=%d buffer=%p\n", buf->index,
		dev_ops_priv->out_buffers[buf->index].buffer);

	/* Rewrap an input buffer with the just size of bytesused
	   because it will be regarded as having data filled to the entire
	   buffer size internally in the GStreame pipeline.
	   Also set the destructor (notify_unref()). */

	if (!gst_buffer_map(dev_ops_priv->out_buffers[buf->index].buffer, &info,
			    GST_MAP_READ)) {
		fprintf(stderr, "Failed to map buffer (%p)\n",
			dev_ops_priv->out_buffers[buf->index].buffer);
		errno = EINVAL;
		return -1;
	}

	buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, info.data,
					     buf->m.planes[0].bytesused, 0,
					     buf->m.planes[0].bytesused,
					     dev_ops_priv->
					     out_buffers[buf->index].buffer,
					     notify_unref);

	gst_buffer_unmap(dev_ops_priv->out_buffers[buf->index].buffer, &info);

	DBG_LOG("buffer rewrap ts=%ld\n", buf->timestamp.tv_sec);
	GST_BUFFER_PTS(buffer) = GST_TIMEVAL_TO_TIME(buf->timestamp);

	flow_ret = gst_app_src_push_buffer(
			GST_APP_SRC(dev_ops_priv->appsrc), buffer);
	if (flow_ret != GST_FLOW_OK) {
		fprintf(stderr,
			"Failed to push a buffer to the pipeline on OUTPUT"
			"(index=%d)\n", buf->index);
		errno = EINVAL;
		return -1;
	}

	return 0;
}

static gboolean
push_to_cap_buffers_queue(struct v4l_gst_priv *dev_ops_priv, GstBuffer *buffer)
{
	gboolean is_empty;
	gint index;

	index = g_queue_index(dev_ops_priv->reqbufs_queue, buffer);
	if (index < 0)
		return FALSE;

	g_mutex_lock(&dev_ops_priv->queue_mutex);

	is_empty = g_queue_is_empty(dev_ops_priv->cap_buffers_queue);
	g_queue_push_tail(dev_ops_priv->cap_buffers_queue, buffer);

	if (is_empty)
		g_cond_signal(&dev_ops_priv->queue_cond);

	g_mutex_unlock(&dev_ops_priv->queue_mutex);

	g_queue_pop_nth_link(dev_ops_priv->reqbufs_queue, index);

	return TRUE;
}

static int
qbuf_ioctl_cap(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	if (!check_v4l2_buffer(buf, dev_ops_priv->cap_buffers,
			       dev_ops_priv->cap_buffers_num,
			       dev_ops_priv->sink_pool))
		return -1;

	/* The buffers in reqbufs_queue, which are pushed by the REQBUF ioctl
	   on CAPTURE, have already contained decoded frames.
	   They should not back to the buffer pool and prepare to be
	   dequeued as they are. */
	if (g_queue_get_length(dev_ops_priv->reqbufs_queue) > 0) {
		DBG_LOG("push_to_cap_buffers_queue index=%d\n", buf->index);
		if (push_to_cap_buffers_queue(dev_ops_priv, dev_ops_priv->
					      cap_buffers[buf->index].buffer))
			return 0;
	}

	DBG_LOG("unref buffer: %p, index=%d\n",
		dev_ops_priv->cap_buffers[buf->index].buffer, buf->index);
	gst_buffer_unref(dev_ops_priv->cap_buffers[buf->index].buffer);

	return 0;
}

static int
qbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	int ret;
	struct v4l_gst_buffer *buffer;

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		buffer = &dev_ops_priv->out_buffers[buf->index];
		gst_buffer_unmap(buffer->buffer, &buffer->info);
		ret = qbuf_ioctl_out(dev_ops_priv, buf);
		if (ret < 0)
			return ret;

	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		buffer = &dev_ops_priv->cap_buffers[buf->index];
		gst_buffer_unmap(buffer->buffer, &buffer->info);
		ret = qbuf_ioctl_cap(dev_ops_priv, buf);
		if (ret < 0)
			return 0;

	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		return -1;
	}

	memset(&buffer->info, 0, sizeof(buffer->info));

	return ret;
}

static inline guint
calc_plane_size(GstVideoInfo *info, GstVideoMeta *meta, gint index)
{
	return meta->stride[index] * GST_VIDEO_INFO_COMP_HEIGHT(info, index);
}

static void
set_v4l2_buffer_plane_params(struct v4l_gst_buffer *buffers, guint n_planes,
			     GstVideoInfo *info, GstVideoMeta *meta,
			     guint bytesused[], struct timeval *timestamp,
			     struct v4l2_buffer *buf)
{
	gint i;

	/* set each plane size */
	if (info && meta)
		/* have video meta information
		   (the decoded data case expected) */
		for (i = 0; i < n_planes; i++)
			buf->m.planes[i].length =
					calc_plane_size(info, meta, i);
	else
		/* no video meta information
		   (the encoded data case expected) */
		buf->m.planes[0].length =
				gst_buffer_get_size(buffers[buf->index].buffer);

	/* set params other than plane sizes */
	for (i = 0; i < n_planes; i++) {
		buf->m.planes[i].m.mem_offset =
				buffers[buf->index].buf_id[i];
		buf->m.planes[i].data_offset = 0;

		if (bytesused)
			buf->m.planes[i].bytesused = bytesused[i];
		else
			buf->m.planes[i].bytesused = 0;
	}

	if (timestamp) {
		buf->timestamp.tv_sec = timestamp->tv_sec;
		buf->timestamp.tv_usec = timestamp->tv_usec;
	} else
		buf->timestamp.tv_sec = buf->timestamp.tv_usec = 0;
}

static int
fill_v4l2_buffer(struct v4l_gst_priv *dev_ops_priv, GstBufferPool *pool,
		 struct v4l_gst_buffer *buffers, gint buffers_num,
		 guint bytesused[], struct timeval *timestamp,
		 struct v4l2_buffer *buf)
{
	GstVideoMeta *meta = NULL;
	guint n_planes;
	GstVideoInfo info;

	get_raw_video_params(pool, buffers[buf->index].buffer, &info, &meta);

	n_planes = (meta) ? meta->n_planes : 1;

	set_v4l2_buffer_plane_params(buffers, n_planes, &info, meta, bytesused,
				     timestamp, buf);

	/* set unused params */
	memset(&buf->timecode, 0, sizeof(buf->timecode));
	buf->sequence = 0;
	buf->flags = 0;
	buf->field = V4L2_FIELD_NONE;

	buf->length = n_planes;

	return 0;
}

static guint
get_v4l2_buffer_index(struct v4l_gst_buffer *buffers, gint buffers_num,
		      GstBuffer *buffer)
{
	gint i;
	guint index = G_MAXUINT;

	for (i = 0; i < buffers_num; i++)
		if (buffers[i].buffer == buffer) {
			index = i;
			break;
		}

	return index;
}

static inline gboolean
is_pipeline_started(GstElement *pipeline)
{
	if (GST_STATE(pipeline) != GST_STATE_NULL)
		return TRUE;

	return FALSE;
}

static GstBuffer *
dequeue_blocking(struct v4l_gst_priv *dev_ops_priv, GQueue *queue, GCond *cond)
{
	GstBuffer *buffer;

	buffer = g_queue_pop_head(queue);
	while (!buffer && is_pipeline_started(dev_ops_priv->pipeline)) {
		g_mutex_unlock(&dev_ops_priv->dev_lock);
		g_cond_wait(cond, &dev_ops_priv->queue_mutex);
		g_mutex_lock(&dev_ops_priv->dev_lock);
		buffer = g_queue_pop_head(queue);
	}

	return buffer;
}

static GstBuffer *
dequeue_non_blocking(GQueue *queue)
{
	GstBuffer *buffer;

	buffer = g_queue_pop_head(queue);
	if (!buffer) {
		DBG_LOG("The buffer pool is empty in "
			"the non-blocking mode, return EAGAIN\n");
		errno = EAGAIN;
	}

	return buffer;
}

static GstBuffer *
dequeue_buffer(struct v4l_gst_priv *dev_ops_priv, GQueue *queue, GCond *cond)
{
	GstBuffer *buffer = NULL;
	guint64 buf;

	g_mutex_lock(&dev_ops_priv->queue_mutex);

	if (dev_ops_priv->is_non_blocking)
		buffer = dequeue_non_blocking(queue);
	else
		buffer = dequeue_blocking(dev_ops_priv, queue, cond);

	if (buffer && g_queue_is_empty(dev_ops_priv->out_buffers_queue) &&
	    g_queue_is_empty(dev_ops_priv->cap_buffers_queue) &&
	    !dev_ops_priv->is_cap_fmt_unacquired) {
		DBG_LOG("Remove POLLIN\n");
		SYS_READ(dev_ops_priv->plugin_fd, &buf, sizeof(buf));
	}

	g_mutex_unlock(&dev_ops_priv->queue_mutex);

	return buffer;
}

static int
dqbuf_ioctl_out(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	GstBuffer *buffer;
	guint index;

	if (!check_no_index_v4l2_buffer(buf, dev_ops_priv->out_buffers,
					dev_ops_priv->src_pool))
		return -1;

	buffer = dequeue_buffer(dev_ops_priv, dev_ops_priv->out_buffers_queue,
				&dev_ops_priv->out_queue_cond);
	if (!buffer)
		return -1;

	index = get_v4l2_buffer_index(dev_ops_priv->out_buffers,
				      dev_ops_priv->out_buffers_num, buffer);
	if (index >= dev_ops_priv->out_buffers_num) {
		fprintf(stderr, "Failed to get a valid buffer index "
			"on OUTPUT\n");
		errno = EINVAL;
		return -1;
	}

	buf->index = index;

	DBG_LOG("success dequeue buffer index=%d buffer=%p\n", index, buffer);

	return fill_v4l2_buffer(dev_ops_priv, dev_ops_priv->src_pool,
				dev_ops_priv->out_buffers,
				dev_ops_priv->out_buffers_num,
				NULL, NULL, buf);
}

static int
dqbuf_ioctl_cap(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	GstBuffer *buffer;
	guint index;
	struct timeval timestamp;
	GstVideoInfo info;
	GstVideoMeta *meta;
	guint bytesused[GST_VIDEO_MAX_PLANES];
	gint i;

	if (!check_no_index_v4l2_buffer(buf, dev_ops_priv->cap_buffers,
					dev_ops_priv->sink_pool))
		return -1;

	buffer = dequeue_buffer(dev_ops_priv, dev_ops_priv->cap_buffers_queue,
				&dev_ops_priv->queue_cond);
	if (!buffer)
		return -1;

	index = get_v4l2_buffer_index(dev_ops_priv->cap_buffers,
				      dev_ops_priv->cap_buffers_num, buffer);
	if (index >= dev_ops_priv->cap_buffers_num) {
		fprintf(stderr, "Failed to get a valid buffer index "
			"on CAPTURE\n");
		errno = EINVAL;
		gst_buffer_unref(buffer);
		return -1;
	}

	buf->index = index;

	if (!get_raw_video_params(dev_ops_priv->sink_pool, buffer, &info,
				  &meta)) {
		fprintf(stderr, "Failed to get video meta data\n");
		errno = EINVAL;
		gst_buffer_unref(buffer);
		return -1;
	}

	/* bytesused should be set to the written data size, but
	   gstvideometa information does not provide it.
	   Alternatively we set the whole memory size for every plane,
	   assuming that it has been filled to the entire size. */
	for (i = 0; i < meta->n_planes; i++)
		bytesused[i] = calc_plane_size(&info, meta, i);

	GST_TIME_TO_TIMEVAL(GST_BUFFER_PTS(buffer), timestamp);

	DBG_LOG("success dequeue buffer index=%d buffer=%p ts=%ld\n",
		index, buffer, timestamp.tv_sec);

	return fill_v4l2_buffer(dev_ops_priv, dev_ops_priv->sink_pool,
				dev_ops_priv->cap_buffers,
				dev_ops_priv->cap_buffers_num,
				bytesused, &timestamp, buf);
}

static int
dqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	int ret;
	struct v4l_gst_buffer *buffer;

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = dqbuf_ioctl_out(dev_ops_priv, buf);
		if (ret < 0)
			return ret;

		buffer = &dev_ops_priv->out_buffers[buf->index];
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = dqbuf_ioctl_cap(dev_ops_priv, buf);
		if (ret < 0)
			return ret;

		buffer = &dev_ops_priv->cap_buffers[buf->index];
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		return -1;
	}

	if (!gst_buffer_map(buffer->buffer, &buffer->info, buffer->flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n", buffer->buffer);
		errno = EINVAL;
		return -1;
	}

	return ret;
}

static int
querybuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	struct v4l_gst_buffer *buffers;
	gint buffers_num;
	GstBufferPool *pool;

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		buffers = dev_ops_priv->out_buffers;
		buffers_num = dev_ops_priv->out_buffers_num;
		pool = dev_ops_priv->src_pool;
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		buffers = dev_ops_priv->cap_buffers;
		buffers_num = dev_ops_priv->cap_buffers_num;
		pool = dev_ops_priv->sink_pool;
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		return -1;
	}

	if (!check_v4l2_buffer(buf, buffers, buffers_num, pool))
		return -1;

	return fill_v4l2_buffer(dev_ops_priv, pool, buffers, buffers_num,
				NULL, NULL, buf);
}

static GstCaps *
get_codec_caps_from_fourcc(guint fourcc)
{
	const gchar *mime;

	mime = convert_codec_type_v4l2_to_gst(fourcc);
	if (!mime) {
		fprintf(stderr,
			"Failed to convert from fourcc to mime string\n");
		return NULL;
	}

	if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_H264) == 0)
		return gst_caps_new_simple(mime, "stream-format",
					   G_TYPE_STRING, "byte-stream", NULL);

	return gst_caps_new_empty_simple(mime);
}

static gsize
set_buffer_id(struct v4l_gst_buffer *buffer, GstBufferPool *pool, gsize offset)
{
	GstVideoInfo info;
	GstVideoMeta *meta;
	gint i;

	if (!get_raw_video_params(pool, buffer->buffer, &info, &meta)) {
		/* deal with this as a single plane */
		buffer->buf_id[0] = offset;
		return gst_buffer_get_size(buffer->buffer) + offset;
	}

	if (meta)
		for (i = 0; i < meta->n_planes; i++) {
			buffer->buf_id[i] = offset;
			offset += calc_plane_size(&info, meta, i);
		}

	return offset;
}

static guint
alloc_buffers_from_pool(GstBufferPool *pool, struct v4l_gst_buffer **buffers)
{

	GstBufferPoolAcquireParams params = { 0, };
	GstFlowReturn flow_ret;
	guint actual_max_buffers;
	struct v4l_gst_buffer *bufs_list;
	gsize id_val = 0;
	gint i;

	if (!gst_buffer_pool_set_active(pool, TRUE)) {
		fprintf(stderr, "Failed to activate buffer pool on OUTPUT\n");
		errno = EINVAL;
		return 0 ;
	}

	/* The buffer pool parameters can not be changed after activation,
	   so it is good time to confirm the number of buffers actually set to
	   the buffer pool. */
	get_buffer_pool_params(pool, NULL, NULL, NULL, &actual_max_buffers);
	if (actual_max_buffers == 0) {
		fprintf(stderr,
			"Cannot handle the unlimited amount of buffers\n");
		errno = EINVAL;
		goto inactivate_pool;
	}

	bufs_list = g_new0(struct v4l_gst_buffer, actual_max_buffers);

	for (i = 0; i < actual_max_buffers; i++) {
		flow_ret = gst_buffer_pool_acquire_buffer(pool,
							  &bufs_list[i].buffer,
							  &params);
		if (flow_ret != GST_FLOW_OK) {
			fprintf(stderr, "Failed to acquire a buffer on OUTPUT\n");
			errno = ENOMEM;
			goto free_bufs_list;
		}

		/* Set identifiers for associating a GstBuffer with
		   a V4L2 buffer in the V4L2 caller side. */
		id_val = set_buffer_id(&bufs_list[i], pool, id_val);

		DBG_LOG("out gst_buffer[%d] : %p\n", i, bufs_list[i].buffer);
	}

	*buffers = bufs_list;

	DBG_LOG("The number of buffers actually set to the buffer pool is %d\n",
		actual_max_buffers);

	return actual_max_buffers;

	/* error cases */
free_bufs_list:
	for (i = 0; i < actual_max_buffers; i++)
		if (bufs_list[i].buffer)
			gst_buffer_unref(bufs_list[i].buffer);
	g_free(bufs_list);
inactivate_pool:
	gst_buffer_pool_set_active(pool, FALSE);

	return 0;
}

static int
reqbuf_ioctl_out(struct v4l_gst_priv *dev_ops_priv,
		 struct v4l2_requestbuffers *req)
{
	GstCaps *caps;
	guint adjusted_count;
	guint allocated_num;

	if (is_pipeline_started(dev_ops_priv->pipeline)) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		return -1;
	}

	if (!is_supported_memory_io(req->memory)) {
		fprintf(stderr, "Only V4L2_MEMORY_MMAP is supported\n");
		return -1;
	}

	if (req->count == 0) {
		/* Buffers that have not been returned to the pool are not
		   freed immediately after the inactivation. In that case,
		   they will be freed as soon as they come back to the pool. */
		if (!gst_buffer_pool_set_active(dev_ops_priv->src_pool, FALSE)) {
			fprintf(stderr, "Failed to inactivate buffer pool, "
				"so could not free buffers\n");
			errno = EBUSY;
			return -1;
		}

		if (dev_ops_priv->out_buffers) {
			g_free(dev_ops_priv->out_buffers);
			dev_ops_priv->out_buffers = NULL;
		}
	}

	if (gst_buffer_pool_is_active(dev_ops_priv->src_pool))
		if (!gst_buffer_pool_set_active(dev_ops_priv->src_pool, FALSE)) {
			fprintf(stderr, "Failed to inactivate buffer pool\n");
			errno = EBUSY;
			return -1;
		}

	caps = get_codec_caps_from_fourcc(dev_ops_priv->out_fourcc);
	if (!caps) {
		errno = EINVAL;
		return -1;
	}

	adjusted_count = MIN(req->count, VIDEO_MAX_FRAME);

	set_buffer_pool_params(dev_ops_priv->src_pool, caps,
			       dev_ops_priv->out_buf_size, adjusted_count,
			       adjusted_count);

	allocated_num = alloc_buffers_from_pool(dev_ops_priv->src_pool,
						&dev_ops_priv->out_buffers);
	if (allocated_num == 0) {
		gst_caps_unref(caps);
		return -1;
	}

	req->count = dev_ops_priv->out_buffers_num = allocated_num;

	DBG_LOG("buffers count=%d\n", req->count);

	return 0;
}

static GstBuffer *
peek_first_cap_buffer(struct v4l_gst_priv *dev_ops_priv)
{
	GstBuffer *buffer;

	g_mutex_lock(&dev_ops_priv->queue_mutex);
	buffer = g_queue_peek_head(dev_ops_priv->reqbufs_queue);
	while (!buffer) {
		g_cond_wait(&dev_ops_priv->queue_cond,
			    &dev_ops_priv->queue_mutex);
		buffer = g_queue_peek_head(dev_ops_priv->reqbufs_queue);
	}
	g_mutex_unlock(&dev_ops_priv->queue_mutex);

	return buffer;
}

static void
wait_for_all_bufs_collected(struct v4l_gst_priv *dev_ops_priv,
			    guint max_buffers)
{
	g_mutex_lock(&dev_ops_priv->queue_mutex);
	while (g_queue_get_length(dev_ops_priv->reqbufs_queue) <
	       max_buffers)
		g_cond_wait(&dev_ops_priv->queue_cond,
			    &dev_ops_priv->queue_mutex);
	g_mutex_unlock(&dev_ops_priv->queue_mutex);
}

static guint
get_next_buf_id(struct v4l_gst_priv *dev_ops_priv)
{
	struct v4l_gst_buffer *last_buffer;

	last_buffer = &dev_ops_priv->
			out_buffers[dev_ops_priv->out_buffers_num - 1];
	return last_buffer->buf_id[0] +
			gst_buffer_get_size(last_buffer->buffer);
}

static gboolean
retrieve_cap_frame_info(GstBufferPool *pool, GstBuffer *buffer,
			struct v4l2_pix_format_mplane *cap_pix_fmt)
{
	GstVideoInfo info;
	GstVideoMeta *meta;
	gint i;

	if (!get_raw_video_params(pool, buffer, &info, &meta)) {
		fprintf(stderr, "Failed to get video meta data\n");
		return FALSE;
	}

	for (i = 0; i < meta->n_planes; i++) {
		cap_pix_fmt->plane_fmt[i].sizeimage =
				calc_plane_size(&info, meta, i);
		cap_pix_fmt->plane_fmt[i].bytesperline = meta->stride[i];
	}

	return TRUE;
}

static guint
create_cap_buffers_list(struct v4l_gst_priv *dev_ops_priv)
{
	GstBuffer *first_buffer;
	guint actual_max_buffers;
	gsize id_val;
	gint i;

	if (dev_ops_priv->cap_buffers)
		/* Cannot realloc the buffers without stopping the pipeline,
		   so return the same number of the buffers so far. */
		return dev_ops_priv->cap_buffers_num;

	g_mutex_unlock(&dev_ops_priv->dev_lock);

	first_buffer = peek_first_cap_buffer(dev_ops_priv);

	g_mutex_lock(&dev_ops_priv->dev_lock);

	if (!first_buffer->pool) {
		fprintf(stderr,
			"Cannot handle buffers not belonging to "
			"a bufferpool\n");
		errno = EINVAL;
		return 0;
	}

	if (dev_ops_priv->sink_pool != first_buffer->pool) {
		DBG_LOG("The buffer pool we prepared is not used by "
			"the pipeline, so replace it with the pool that is "
			"actually used\n");
		gst_object_unref(dev_ops_priv->sink_pool);
		dev_ops_priv->sink_pool = gst_object_ref(first_buffer->pool);
	}

	/* Confirm the number of buffers actually set to the buffer pool. */
	get_buffer_pool_params(dev_ops_priv->sink_pool, NULL, NULL, NULL,
			       &actual_max_buffers);
	if (actual_max_buffers == 0) {
		fprintf(stderr,
			"Cannot handle the unlimited amount of buffers\n");
		errno = EINVAL;
		return 0;
	}

	if (!retrieve_cap_frame_info(dev_ops_priv->sink_pool, first_buffer,
				     &dev_ops_priv->cap_pix_fmt)) {
		fprintf(stderr, "Failed to retrieve frame info on CAPTURE\n");
		errno = EINVAL;
		return 0;
	}

	g_mutex_unlock(&dev_ops_priv->dev_lock);

	/* We wait for buffers from appsink to be collected for
	   the maximum number of the buffer pool. */
	wait_for_all_bufs_collected(dev_ops_priv, actual_max_buffers);

	g_mutex_lock(&dev_ops_priv->dev_lock);

	dev_ops_priv->cap_buffers = g_new0(struct v4l_gst_buffer,
					   actual_max_buffers);

	id_val = get_next_buf_id(dev_ops_priv);
	for (i = 0; i < actual_max_buffers; i++) {
		dev_ops_priv->cap_buffers[i].buffer =
				g_queue_peek_nth(dev_ops_priv->
						 reqbufs_queue, i);

		/* Set identifiers for associating a GstBuffer with
		   a V4L2 buffer in the V4L2 caller side. */
		id_val = set_buffer_id(&dev_ops_priv->cap_buffers[i],
				       dev_ops_priv->sink_pool, id_val);

		DBG_LOG("cap gst_buffer[%d] : %p\n", i,
			dev_ops_priv->cap_buffers[i].buffer);
	}

	DBG_LOG("The number of buffers actually set to the buffer pool is %d\n",
		actual_max_buffers);

	return actual_max_buffers;
}

static int
reqbuf_ioctl_cap(struct v4l_gst_priv *dev_ops_priv,
		 struct v4l2_requestbuffers *req)
{
	guint buffers_num;

	if (!is_supported_memory_io(req->memory)) {
		fprintf(stderr, "Only V4L2_MEMORY_MMAP is supported\n");
		return -1;
	}

	if (req->count == 0)
		/* The buffers on CAPTURE are managed by the GStreamer pipeline
		   as a buffer pool and can be freed by stopping
		   the pipeline only. For the general V4L2 API behavior to
		   the caller side, this case just returns success as if
		   the buffers were actually freed. */
		return 0;

	if (!is_pipeline_started(dev_ops_priv->pipeline)) {
		fprintf(stderr,
			"Need to start the pipeline for the buffer request "
			"on CAPTURE\n");
		errno = EINVAL;
		return -1;
	}

	g_mutex_lock(&dev_ops_priv->cap_reqbuf_mutex);
	dev_ops_priv->cap_buffers_num = MIN(req->count, VIDEO_MAX_FRAME);
	g_cond_signal(&dev_ops_priv->cap_reqbuf_cond);
	g_mutex_unlock(&dev_ops_priv->cap_reqbuf_mutex);

	buffers_num = create_cap_buffers_list(dev_ops_priv);
	if (buffers_num == 0)
		return -1;

	req->count = dev_ops_priv->cap_buffers_num = buffers_num;

	DBG_LOG("buffers count=%d\n", req->count);

	return 0;
}

static int
reqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_requestbuffers *req)
{
	int ret;

	if (req->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = reqbuf_ioctl_out(dev_ops_priv, req);
	else if (req->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = reqbuf_ioctl_cap(dev_ops_priv, req);
	else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

static gboolean
relink_elements_with_caps_filtered(GstElement *src_elem, GstElement *dest_elem,
				   GstCaps *caps)
{
	gst_element_unlink(src_elem, dest_elem);
	return gst_element_link_filtered(src_elem, dest_elem, caps);
}

static gboolean
set_out_format_to_pipeline(struct v4l_gst_priv *dev_ops_priv)
{
	GstCaps *caps;

	caps = get_codec_caps_from_fourcc(dev_ops_priv->out_fourcc);
	if (!caps) {
		errno = EINVAL;
		return FALSE;
	}

	gst_app_src_set_caps(GST_APP_SRC(dev_ops_priv->appsrc), caps);
	gst_caps_unref(caps);

	return TRUE;
}

static gboolean
set_cap_format_to_pipeline(struct v4l_gst_priv *dev_ops_priv)
{
	GstElement *peer_elem;
	GstCaps *caps;
	GstVideoFormat fmt;
	gboolean ret;

	fmt = convert_video_format_v4l2_to_gst(dev_ops_priv->
					       cap_pix_fmt.pixelformat);
	if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
		fprintf(stderr, "Invalid format on CAPTURE\n");
		errno = EINVAL;
		return FALSE;
	}

	caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
				   gst_video_format_to_string(fmt), NULL);

	peer_elem = get_peer_element(dev_ops_priv->appsink, "sink");
	if (!relink_elements_with_caps_filtered(peer_elem,
						dev_ops_priv->appsink, caps)) {
		fprintf(stderr, "Failed to relink elements with "
			"the CAPTURE setting (caps=%s)\n",
			gst_caps_to_string(caps));
		errno = EINVAL;
		ret = FALSE;
		goto free_objects;
	}

	ret = TRUE;

free_objects:
	gst_caps_unref(caps);
	gst_object_unref(peer_elem);

	return ret;
}

static gpointer
out_queue_thread_func(gpointer data)
{
	struct v4l_gst_priv *dev_ops_priv = data;
	GstFlowReturn flow_ret;
	GstBuffer *buffer;
	GstBufferPoolAcquireParams params = { 0, };
	gboolean is_empty;
	guint64 buf = 1;
#ifdef DEBUG
	guint index;
#endif

	flow_ret = gst_buffer_pool_acquire_buffer(dev_ops_priv->src_pool,
						  &buffer, &params);
	while (flow_ret == GST_FLOW_OK) {
		/* The buffer held at the following will be released
		   right after the cap format has been acquirable.
		   Please see the comment for the detail in
		   pad_probe_query(). */
		g_mutex_lock(&dev_ops_priv->pending_buf_mutex);
		if (!g_atomic_int_get(&dev_ops_priv->is_cap_fmt_acquirable) &&
		    !dev_ops_priv->pending_buffer) {
			DBG_LOG("Holding the first acquired buffer "
				"on OUTPUT\n");
			dev_ops_priv->pending_buffer = buffer;
			g_mutex_unlock(&dev_ops_priv->pending_buf_mutex);
			goto skip_queuing;
		}
		g_mutex_unlock(&dev_ops_priv->pending_buf_mutex);

		g_mutex_lock(&dev_ops_priv->queue_mutex);

		is_empty = g_queue_is_empty(dev_ops_priv->out_buffers_queue);
		g_queue_push_tail(dev_ops_priv->out_buffers_queue, buffer);

		if (is_empty &&
		    !GST_BUFFER_POOL_IS_FLUSHING(dev_ops_priv->src_pool)) {
			g_cond_signal(&dev_ops_priv->out_queue_cond);
			DBG_LOG("Emit POLLIN\n");
			SYS_WRITE(dev_ops_priv->plugin_fd, &buf, sizeof(buf));
		}

		g_mutex_unlock(&dev_ops_priv->queue_mutex);

skip_queuing:
		flow_ret =
			gst_buffer_pool_acquire_buffer(dev_ops_priv->src_pool,
						       &buffer, &params);

#ifdef DEBUG
		index = get_v4l2_buffer_index(dev_ops_priv->out_buffers,
					      dev_ops_priv->out_buffers_num,
					      buffer);
#endif
		DBG_LOG("return intput buffer index:%d buffer:%p\n", index,
			buffer);
	}

	return NULL;
}

static int
streamon_ioctl_out(struct v4l_gst_priv *dev_ops_priv)
{
	if (is_pipeline_started(dev_ops_priv->pipeline)) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		return -1;
	}

	if (!set_out_format_to_pipeline(dev_ops_priv))
		return -1;
	if (!set_cap_format_to_pipeline(dev_ops_priv))
		return -1;

	dev_ops_priv->out_queue_thread =
			g_thread_new(NULL, out_queue_thread_func, dev_ops_priv);

	gst_element_set_state(dev_ops_priv->pipeline, GST_STATE_PLAYING);

	return 0;
}

static int
streamon_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type)
{
	int ret;

	if (*type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		DBG_LOG("streamon on OUTPUT\n");
		ret = streamon_ioctl_out(dev_ops_priv);
	} else if (*type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		/* no processing */
		ret = 0;
	else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

static int
streamoff_ioctl_out(struct v4l_gst_priv *dev_ops_priv)
{
	gst_element_set_state(dev_ops_priv->pipeline, GST_STATE_NULL);

	gst_buffer_pool_set_flushing(dev_ops_priv->src_pool, TRUE);
	g_thread_join(dev_ops_priv->out_queue_thread);
	gst_buffer_pool_set_flushing(dev_ops_priv->src_pool, FALSE);

	if (!dev_ops_priv->is_non_blocking) {
		g_cond_broadcast(&dev_ops_priv->out_queue_cond);
		g_cond_broadcast(&dev_ops_priv->queue_cond);
	}

	/* Vacate the buffers queues to make them available in the next time */
	while (dequeue_non_blocking(dev_ops_priv->out_buffers_queue));
	while (dequeue_non_blocking(dev_ops_priv->cap_buffers_queue));

	g_atomic_int_set(&dev_ops_priv->is_cap_fmt_acquirable, 0);
	if (dev_ops_priv->cap_buffers) {
		g_free(dev_ops_priv->cap_buffers);
		dev_ops_priv->cap_buffers = NULL;
	}
	init_decoded_frame_params(&dev_ops_priv->cap_pix_fmt);

	return 0;
}

static int
streamoff_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type)
{
	int ret;

	if (*type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = streamoff_ioctl_out(dev_ops_priv);
	else if (*type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		/* no processing */
		ret = 0;
	else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

static int
subscribe_event_ioctl(struct v4l_gst_priv *dev_ops_priv,
		      struct v4l2_event_subscription *sub)
{
	return 0;
}

static int
dqevent_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_event *ev)
{
	/* TODO: Add the implementation for subscribed event notifications.
		 Always return failure until the feature has been supported. */
	return -1;
}

static int plugin_ioctl(void *dev_ops_priv, int fd,
			unsigned long int cmd, void *arg)
{
	struct v4l_gst_priv *priv = dev_ops_priv;
	int ret = -1;

	(void)fd; /* unused */

	g_mutex_lock(&priv->dev_lock);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		ret = querycap_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_S_FMT:
		ret = set_fmt_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_G_FMT:
		ret = get_fmt_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_ENUM_FMT:
		ret = enum_fmt_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_G_CTRL:
		ret = get_ctrl_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_QBUF:
		ret = qbuf_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_DQBUF:
		ret = dqbuf_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_QUERYBUF:
		ret = querybuf_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_REQBUFS:
		ret = reqbuf_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_STREAMON:
		ret = streamon_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_STREAMOFF:
		ret = streamoff_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_SUBSCRIBE_EVENT:
		ret = subscribe_event_ioctl(dev_ops_priv, arg);
		break;
	case VIDIOC_DQEVENT:
		ret = dqevent_ioctl(dev_ops_priv, arg);
		break;
	default:
		perror("unknown ioctl");
		errno = EINVAL;
		break;
	}

	g_mutex_unlock(&priv->dev_lock);

	return ret;
}

static gint
find_out_buffer_by_offset(struct v4l_gst_priv *dev_ops_priv, int64_t offset)
{
	gint index = -1;
	gint i;

	for (i = 0; i < dev_ops_priv->out_buffers_num; i++)
		if (dev_ops_priv->out_buffers[i].buf_id[0] == offset) {
			index = i;
			break;
		}

	return index;
}

static void *
map_out_buffer(struct v4l_gst_priv *dev_ops_priv, gint index,
	       GstMapFlags flags)
{
	GstMapInfo info;
	void *data;

	if (!gst_buffer_map(dev_ops_priv->out_buffers[index].buffer,
			    &info, flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n",
			dev_ops_priv->out_buffers[index].buffer);
		errno = EINVAL;
		return MAP_FAILED;
	}

	data = info.data;

	gst_buffer_unmap(dev_ops_priv->out_buffers[index].buffer, &info);

	dev_ops_priv->out_buffers[index].flags = flags;

	return data;
}

static gboolean
find_cap_buffer_by_offset(struct v4l_gst_priv *dev_ops_priv, int64_t offset,
			  gint *index, gint *plane)
{
	gboolean ret = FALSE;
	gint i, j;

	for (i = 0; i < dev_ops_priv->cap_buffers_num; i++) {
		for (j = 0; j < dev_ops_priv->cap_pix_fmt.num_planes; j++)
			if (dev_ops_priv->cap_buffers[i].buf_id[j] == offset) {
				*index = i;
				*plane = j;
				ret = TRUE;
				break;
			}

		if (ret)
			break;
	}

	return ret;
}

static void *
map_cap_buffer(struct v4l_gst_priv *dev_ops_priv, gint index, gint plane,
	       GstMapFlags flags)
{
	GstVideoMeta *meta;
	GstMapInfo info;
	void *data;

	if (!gst_buffer_map(dev_ops_priv->cap_buffers[index].buffer,
			    &info, flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n",
			dev_ops_priv->cap_buffers[index].buffer);
		errno = EINVAL;
		return MAP_FAILED;
	}

	if (!get_raw_video_params(dev_ops_priv->sink_pool,
				  dev_ops_priv->cap_buffers[index].buffer,
				  NULL, &meta)) {
		fprintf(stderr, "Failed to get video meta data\n");
		errno = EINVAL;
		gst_buffer_unmap(dev_ops_priv->cap_buffers[index].buffer,
				 &dev_ops_priv->cap_buffers[index].info);
		return MAP_FAILED;
	}

	data = info.data + meta->offset[plane];

	gst_buffer_unmap(dev_ops_priv->cap_buffers[index].buffer, &info);

	dev_ops_priv->cap_buffers[index].flags = flags;

	return data;
}

static void *
plugin_mmap(void *dev_ops_priv, void *start, size_t length, int prot,
	    int flags, int fd, int64_t offset)
{
	struct v4l_gst_priv *priv = dev_ops_priv;
	GstMapFlags map_flags;
	gint index;
	gint plane;
	void *map = MAP_FAILED;

	/* unused */
	(void)start;
	(void)flags;
	(void)fd;

	/* The GStreamer memory mapping internally maps
	   the whole allocated size of a buffer, so the mapping length
	   does not need to be specified. */
	(void)length;

	map_flags = (prot & PROT_READ) ? GST_MAP_READ : 0;
	map_flags |= (prot & PROT_WRITE) ? GST_MAP_WRITE : 0;

	g_mutex_lock(&priv->dev_lock);

	index = find_out_buffer_by_offset(priv, offset);
	if (index >= 0) {
		map = map_out_buffer(priv, index, flags);
		goto unlock;
	}

	if (find_cap_buffer_by_offset(priv, offset, &index, &plane)) {
		map = map_cap_buffer(priv, index, plane, flags);
		goto unlock;
	}

unlock:
	g_mutex_unlock(&priv->dev_lock);

	return map;
}

PLUGIN_PUBLIC const struct libv4l_dev_ops libv4l2_plugin = {
	.init = &plugin_init,
	.close = &plugin_close,
	.ioctl = &plugin_ioctl,
	.mmap = &plugin_mmap,
};
