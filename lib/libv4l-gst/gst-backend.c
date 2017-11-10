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
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

#include "libv4l-gst-bufferpool.h"

#include "gst-backend.h"
#include "evfd-ctrl.h"
#include "debug.h"

#define DEF_CAP_MIN_BUFFERS		2

enum buffer_state {
	V4L_GST_BUFFER_INIT = 0,
	V4L_GST_BUFFER_QUEUED,
	V4L_GST_BUFFER_DEQUEUED,
	V4L_GST_BUFFER_READY_FOR_DEQUEUE,
	V4L_GST_BUFFER_PREROLLED,
};

struct v4l_gst_buffer {
	GstBuffer *buffer;
	GstMapInfo info;
	GstMapFlags flags;

	struct v4l2_plane planes[GST_VIDEO_MAX_PLANES];

	struct gst_backend_priv *priv;

	enum buffer_state state;
};

struct fmts {
        guint fmt;
        gchar *fmt_char;
};

struct gst_backend_priv {
	struct v4l_gst_priv *dev_ops_priv;

	GstElement *pipeline;
	GstElement *appsrc;
	GstElement *appsink;

	GstAppSinkCallbacks appsink_cb;

	void *pool_lib_handle;
	struct libv4l_gst_buffer_pool_ops *pool_ops;

	struct fmts *out_fmts;
	gint out_fmts_num;
	struct fmts *cap_fmts;
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
	guint cap_buffers_num;
	guint cap_buffers_req;
	gint confirmed_bufcount;

	int64_t mmap_offset;

	GQueue *cap_buffers_queue;
	GMutex queue_mutex;
	GCond queue_cond;

	gint empty_out_buffer_cnt;

#ifdef ENABLE_CHROMIUM_COMPAT
	GstBuffer *pending_buffer;
#endif
	gulong probe_id;

	GMutex cap_reqbuf_mutex;
	GCond cap_reqbuf_cond;

	gboolean is_cap_fmt_acquirable;

	gboolean is_pipeline_started;

	GMutex dev_lock;

	GstBuffer *eos_buffer;

	gint max_width;
	gint max_height;
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
	{ V4L2_PIX_FMT_RGB32, GST_VIDEO_FORMAT_BGRA },
	{ V4L2_PIX_FMT_BGR32, GST_VIDEO_FORMAT_ARGB },
	{ V4L2_PIX_FMT_NV12MT, GST_VIDEO_FORMAT_NV12_64Z32 },
};

static gboolean
retrieve_cap_frame_info(GstBufferPool *pool, GstBuffer *buffer,
			struct v4l2_pix_format_mplane *cap_pix_fmt);

static gboolean
parse_conf_settings(gchar **pipeline_str, gchar **pool_lib_path,
		    gint *min_buffers, gint *max_width, gint *max_height)
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

                *max_width = g_key_file_get_integer(conf_key, groups[i],
                                                      "max-width", NULL);
                *max_height = g_key_file_get_integer(conf_key, groups[i],
                                                      "max-height", NULL);
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

	/* This dynamic linking will keep loaded even after the plugin has been
	   closed in order to prevent from the duplicate class registration of
	   the buffer pool due to the static variable that indicates if
	   the class has already been registered being deleted when the dynamic
	   library is unloaded. */
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

	for (i = 0; i < G_N_ELEMENTS(v4l_gst_vid_fmt_tbl); i++) {
		if (v4l_gst_vid_fmt_tbl[i].format == fmt)
			fourcc = v4l_gst_vid_fmt_tbl[i].fourcc;
	}

	return fourcc;
}

static GstVideoFormat
convert_video_format_v4l2_to_gst(guint fourcc)
{
	gint i;
	GstVideoFormat fmt = GST_VIDEO_FORMAT_UNKNOWN;

	for (i = 0; i < G_N_ELEMENTS(v4l_gst_vid_fmt_tbl); i++) {
		if (v4l_gst_vid_fmt_tbl[i].fourcc == fourcc)
			fmt = v4l_gst_vid_fmt_tbl[i].format;
	}

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
get_supported_video_format_out(GstElement *appsrc, struct fmts **out_fmts,
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
		*out_fmts = g_new0(struct fmts, *out_fmts_num);

		(*out_fmts)[0].fmt = V4L2_PIX_FMT_H264;
		(*out_fmts)[1].fmt = V4L2_PIX_FMT_VP8;
		(*out_fmts)[0].fmt_char = g_strdup("V4L2_PIX_FMT_H264");
		(*out_fmts)[1].fmt_char = g_strdup("V4L2_PIX_FMT_VP8");

		DBG_LOG("out supported codecs : h264, vp8\n");
	} else {
		structure = gst_caps_get_structure(caps, 0);
		mime = gst_structure_get_name(structure);

		if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_H264) == 0) {
			fourcc = V4L2_PIX_FMT_H264;
		} else if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_VP8) == 0) {
			fourcc = V4L2_PIX_FMT_VP8;
		} else {
			fprintf(stderr, "Unsupported codec : %s\n", mime);
			gst_caps_unref(caps);
			return FALSE;
		}

		DBG_LOG("out supported codec : %s\n", mime);

		*out_fmts_num = 1;
		*out_fmts = g_new0(struct fmts, *out_fmts_num);

		(*out_fmts)[0].fmt = fourcc;
		if(fourcc == V4L2_PIX_FMT_H264)
			(*out_fmts)[0].fmt_char = g_strdup("V4L2_PIX_FMT_H264");
		else
			(*out_fmts)[0].fmt_char = g_strdup("V4L2_PIX_FMT_VP8");
	}

	gst_caps_unref(caps);

	return TRUE;
}

static gboolean
get_supported_video_format_cap(GstElement *appsink, struct fmts **cap_fmts,
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
	*cap_fmts = g_new0(struct fmts, list_size + 2);

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

		(*cap_fmts)[fmts_num].fmt = fourcc;
		(*cap_fmts)[fmts_num++].fmt_char = g_strdup(fmt_str);

		/* Add legacy RGB formats */
		if (fourcc == V4L2_PIX_FMT_ARGB32) {
			(*cap_fmts)[fmts_num].fmt = V4L2_PIX_FMT_RGB32;
			(*cap_fmts)[fmts_num++].fmt_char = g_strdup(fmt_str);
		} else if (fourcc == V4L2_PIX_FMT_ABGR32) {
			(*cap_fmts)[fmts_num].fmt = V4L2_PIX_FMT_BGR32;
			(*cap_fmts)[fmts_num++].fmt_char = g_strdup(fmt_str);
		}
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
retrieve_cap_format_info(struct gst_backend_priv *priv, GstVideoInfo *info)
{
	gint fourcc;

	priv->cap_pix_fmt.width = info->width;
	priv->cap_pix_fmt.height = info->height;

	fourcc = convert_video_format_gst_to_v4l2(info->finfo->format);
	if (priv->cap_pix_fmt.pixelformat != 0 &&
	    priv->cap_pix_fmt.pixelformat != fourcc) {
		fprintf(stderr, "WARNING: Unexpected cap video format\n");
	}
	priv->cap_pix_fmt.pixelformat = fourcc;

	priv->cap_pix_fmt.num_planes = info->finfo->n_planes;
}

static void
wait_for_cap_reqbuf_invocation(struct gst_backend_priv *priv)
{
	g_mutex_lock(&priv->cap_reqbuf_mutex);
	while (priv->cap_buffers_req == 0)
		g_cond_wait(&priv->cap_reqbuf_cond, &priv->cap_reqbuf_mutex);
	g_mutex_unlock(&priv->cap_reqbuf_mutex);
}

static inline void
release_out_buffer(struct gst_backend_priv *priv, GstBuffer *buffer)
{
	g_atomic_int_inc(&priv->empty_out_buffer_cnt);
	gst_buffer_unref(buffer);

	DBG_LOG("unref buffer: %p\n", buffer);
	set_event(priv->dev_ops_priv->event_state, POLLIN);
}

static GstPadProbeReturn
pad_probe_query(GstPad *pad, GstPadProbeInfo *probe_info, gpointer user_data)
{
	struct gst_backend_priv *priv = user_data;
	GstQuery *query;
	GstCaps *caps;
	GstVideoInfo info;

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

		retrieve_cap_format_info(priv, &info);

		priv->is_cap_fmt_acquirable = TRUE;

		wait_for_cap_reqbuf_invocation(priv);

		set_buffer_pool_params(priv->sink_pool, caps, info.size,
				       priv->cap_buffers_req,
				       priv->cap_buffers_req);

		gst_query_add_allocation_pool(query, priv->sink_pool,
					      info.size,
					      priv->cap_buffers_req,
					      priv->cap_buffers_req);
	}

	return GST_PAD_PROBE_OK;
}

static gulong
setup_query_pad_probe(struct gst_backend_priv *priv)
{
	GstPad *peer_pad;
	gulong probe_id;

	peer_pad = get_peer_pad(priv->appsink, "sink");
	probe_id = gst_pad_add_probe(peer_pad,
				     GST_PAD_PROBE_TYPE_QUERY_DOWNSTREAM,
				     (GstPadProbeCallback) pad_probe_query,
				     priv, NULL);
	gst_object_unref(peer_pad);

	return probe_id;
}

static GstBuffer *
get_buffer_from_sample(GstSample *sample)
{
	GstBuffer *buffer;

	buffer = gst_sample_get_buffer(sample);
	gst_buffer_ref(buffer);
	gst_sample_unref(sample);

	return buffer;
}

void
appsink_callback_eos(GstAppSink *appsink, gpointer user_data)
{
	struct gst_backend_priv *priv = user_data;
	if (priv->eos_buffer)
		release_out_buffer(priv, priv->eos_buffer);
}

static guint
get_v4l2_buffer_index(struct v4l_gst_buffer *buffers, gint buffers_num,
		      GstBuffer *buffer)
{
	gint i;
	guint index = G_MAXUINT;

	for (i = 0; i < buffers_num; i++) {
		if (buffers[i].buffer == buffer) {
			index = i;
			break;
		}
	}

	return index;
}

static GstFlowReturn
appsink_callback_preroll(GstAppSink *appsink, gpointer user_data)
{
	struct gst_backend_priv *priv = user_data;
	GstSample *sample;
	GstBuffer *buffer;
	guint preroll_idx;
	sample = gst_app_sink_pull_preroll(appsink);
	buffer = get_buffer_from_sample(sample);
	if (!buffer->pool) {
		fprintf(stderr,
			"Cannot handle buffers not belonging to "
			"a bufferpool\n");
		return GST_FLOW_ERROR;
	}

	if (priv->sink_pool != buffer->pool) {
		DBG_LOG("The buffer pool we prepared is not used by "
			"the pipeline, so replace it with the pool that is "
			"actually used\n");
		gst_object_unref(priv->sink_pool);
		priv->sink_pool = gst_object_ref(buffer->pool);
	}

	/* Confirm the number of buffers actually set to the buffer pool. */
	get_buffer_pool_params(priv->sink_pool, NULL, NULL, NULL,
		       &priv->cap_buffers_num);
	if (priv->cap_buffers_num == 0) {
		fprintf(stderr,
			"Cannot handle the unlimited amount of buffers\n");
		return GST_FLOW_ERROR;
	}
	if (!retrieve_cap_frame_info(priv->sink_pool, buffer,
			     &priv->cap_pix_fmt)) {
		fprintf(stderr, "Failed to retrieve frame info on CAPTURE\n");
		return GST_FLOW_ERROR;
	}
	if (!priv->cap_buffers)
		priv->cap_buffers = g_new0(struct v4l_gst_buffer, priv->cap_buffers_num);

	preroll_idx = priv->cap_buffers_num - 1;
	priv->cap_buffers[preroll_idx].buffer = buffer;
	priv->cap_buffers[preroll_idx].state = V4L_GST_BUFFER_PREROLLED;
	gst_buffer_unref(buffer);
	return GST_FLOW_OK;
}

static GstFlowReturn
appsink_callback_new_sample(GstAppSink *appsink, gpointer user_data)
{
	struct gst_backend_priv *priv = user_data;
	GstBuffer *buffer;
	GstSample *sample;
	gboolean is_empty;
	GQueue *queue;
	guint index;
	guint preroll_idx;

	sample = gst_app_sink_pull_sample(appsink);
	buffer = get_buffer_from_sample(sample);


	preroll_idx = priv->cap_buffers_num - 1;

	DBG_LOG("pull buffer: %p\n", buffer);

	//TODO: Check on locking and error conditions. Need to wake up reqbufs thread on error.

	g_mutex_lock(&priv->queue_mutex);


	index = get_v4l2_buffer_index(priv->cap_buffers, priv->confirmed_bufcount,
			buffer);

	if (index >= preroll_idx) {
		int j;
		struct v4l_gst_buffer *buf;
		if (priv->confirmed_bufcount >= priv->cap_buffers_num) {
			fprintf(stderr, "Unknown buffer received from pipline\n");
			g_mutex_unlock(&priv->queue_mutex);
			return GST_FLOW_ERROR;
		}

		if (buffer == priv->cap_buffers[preroll_idx].buffer) {
			gst_buffer_unref(buffer);
			g_cond_signal(&priv->queue_cond);
			g_mutex_unlock(&priv->queue_mutex);
			return GST_FLOW_OK;
		}

		buf = &(priv->cap_buffers[priv->confirmed_bufcount++]);
		buf->buffer = buffer;
		buf->state = V4L_GST_BUFFER_READY_FOR_DEQUEUE;
		for (j = 0; j < priv->cap_pix_fmt.num_planes; j++) {
			buf->planes[j].length =
					priv->cap_pix_fmt.plane_fmt[j].sizeimage;
		}
		g_cond_signal(&priv->queue_cond);
		g_mutex_unlock(&priv->queue_mutex);
		return GST_FLOW_OK;
	}

	if (priv->cap_buffers[index].state == V4L_GST_BUFFER_INIT) {
		priv->cap_buffers[index].state = V4L_GST_BUFFER_READY_FOR_DEQUEUE;
		g_mutex_unlock(&priv->queue_mutex);
		return GST_FLOW_OK;
	}

	queue = priv->cap_buffers_queue;

	is_empty = g_queue_is_empty(queue);
	g_queue_push_tail(queue, buffer);

	if (is_empty) {
		g_cond_signal(&priv->queue_cond);
		set_event(priv->dev_ops_priv->event_state, POLLOUT);
	}

	g_mutex_unlock(&priv->queue_mutex);

	return GST_FLOW_OK;
}

static gboolean
init_app_elements(struct gst_backend_priv *priv)
{
	struct fmts *out_fmts;
	gint out_fmts_num;
	struct fmts *cap_fmts;
	gint cap_fmts_num;
	guint i;

	/* Get appsrc and appsink elements respectively from the pipeline */
	if (!get_app_elements(priv->pipeline, &priv->appsrc, &priv->appsink))
		return FALSE;

	/* Set the appsrc queue size to unlimited.
	   The amount of buffers is managed by the buffer pool. */
	gst_app_src_set_max_bytes(GST_APP_SRC(priv->appsrc), 0);

	priv->appsink_cb.new_sample = appsink_callback_new_sample;
	priv->appsink_cb.eos = appsink_callback_eos;
	priv->appsink_cb.new_preroll = appsink_callback_preroll;

	gst_app_sink_set_callbacks(GST_APP_SINK(priv->appsink),
				   &priv->appsink_cb, priv, NULL);

	/* For queuing buffers received from appsink */
	priv->cap_buffers_queue = g_queue_new();
	g_mutex_init(&priv->queue_mutex);
	g_cond_init(&priv->queue_cond);

	if (!get_supported_video_format_out(priv->appsrc, &out_fmts,
					    &out_fmts_num))
		return FALSE;
	if (!get_supported_video_format_cap(priv->appsink, &cap_fmts,
					    &cap_fmts_num)) {
		for(i = 0; i < out_fmts_num; i++)
                        g_free(out_fmts[i].fmt_char);
		g_free(out_fmts);
		return FALSE;
	}
	priv->out_fmts = out_fmts;
	priv->out_fmts_num = out_fmts_num;
	priv->cap_fmts = cap_fmts;
	priv->cap_fmts_num = cap_fmts_num;

	return TRUE;
}

static gboolean
init_buffer_pool(struct gst_backend_priv *priv, gchar *pool_lib_path)
{
	/* Get the external buffer pool when it is specified in
	   the configuration file */
	if (pool_lib_path) {
		get_buffer_pool_ops(pool_lib_path,
				    &priv->pool_lib_handle, &priv->pool_ops);
	}

	create_buffer_pool(priv->pool_ops, &priv->src_pool, &priv->sink_pool);

	/* To hook allocation queries */
	priv->probe_id = setup_query_pad_probe(priv);
	if (priv->probe_id == 0) {
		fprintf(stderr, "Failed to setup query pad probe\n");
		goto free_pool;
	}

	/* To wait for the requested number of buffers on CAPTURE
	   to be set in pad_probe_query() */
	g_mutex_init(&priv->cap_reqbuf_mutex);
	g_cond_init(&priv->cap_reqbuf_cond);

	return TRUE;

	/* error cases */
free_pool:
	gst_object_unref(priv->src_pool);
	gst_object_unref(priv->sink_pool);

	return FALSE;
}

int
gst_backend_init(struct v4l_gst_priv *dev_ops_priv)
{
	struct gst_backend_priv *priv;
	gchar *pipeline_str = NULL;
	gchar *pool_lib_path = NULL;
	guint i;

	priv = calloc(1, sizeof(*priv));
	if (!priv) {
		perror("Couldn't allocate memory for gst-backend");
		return -1;
	}

	priv->dev_ops_priv = dev_ops_priv;

	if (!parse_conf_settings(&pipeline_str, &pool_lib_path,
				 &priv->cap_min_buffers,
				 &priv->max_width,
				 &priv->max_height))
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

	dev_ops_priv->gst_priv = priv;

	return 0;

	/* error cases */
free_app_elems_init_objs:
	g_queue_free(priv->cap_buffers_queue);
	g_mutex_clear(&priv->queue_mutex);
	g_cond_clear(&priv->queue_cond);

	for(i = 0; i < priv->out_fmts_num; i++)
                g_free(priv->out_fmts[i].fmt_char);
	g_free(priv->out_fmts);
	for(i = 0; i < priv->cap_fmts_num; i++)
                g_free(priv->cap_fmts[i].fmt_char);
	g_free(priv->cap_fmts);
free_pipeline:
	gst_object_unref(priv->pipeline);
free_pool_path:
	g_free(pool_lib_path);
free_priv:
	g_free(priv);

	return -1;
}

static void
remove_query_pad_probe(GstElement *appsink, gulong probe_id)
{
	GstPad *peer_pad;

	peer_pad = get_peer_pad(appsink, "sink");
	gst_pad_remove_probe(peer_pad, probe_id);
	gst_object_unref(peer_pad);
}

void
gst_backend_deinit(struct v4l_gst_priv *dev_ops_priv)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	guint i;

	g_mutex_clear(&priv->dev_lock);

	remove_query_pad_probe(priv->appsink, priv->probe_id);

	if (priv->out_buffers)
		g_free(priv->out_buffers);

	if (priv->cap_buffers)
		g_free(priv->cap_buffers);


	gst_object_unref(priv->src_pool);
	gst_object_unref(priv->sink_pool);

	for(i = 0; i < priv->out_fmts_num; i++)
                g_free(priv->out_fmts[i].fmt_char);
	g_free(priv->out_fmts);
	for(i = 0; i < priv->cap_fmts_num; i++)
                g_free(priv->cap_fmts[i].fmt_char);
	g_free(priv->cap_fmts);

	g_queue_free(priv->cap_buffers_queue);
	g_mutex_clear(&priv->queue_mutex);
	g_cond_clear(&priv->queue_cond);

	g_mutex_clear(&priv->cap_reqbuf_mutex);
	g_cond_clear(&priv->cap_reqbuf_cond);

	gst_object_unref(priv->pipeline);
}

int
querycap_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_capability *cap)
{
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE
#ifdef ENABLE_CHROMIUM_COMPAT
			| V4L2_CAP_VIDEO_CAPTURE_MPLANE
			| V4L2_CAP_VIDEO_OUTPUT_MPLANE
#endif
			| V4L2_CAP_EXT_PIX_FORMAT
			| V4L2_CAP_STREAMING;

	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	g_strlcpy((gchar *)cap->driver, "libv4l-gst", sizeof(cap->driver));
	g_strlcpy((gchar *)cap->card, "gst-dummy", sizeof(cap->card));
	g_strlcpy((gchar *)cap->bus_info, "user-vst-gst-000", sizeof(cap->bus_info));
	memset(cap->reserved, 0, sizeof(cap->reserved));

	return 0;
}

static gboolean
is_pix_fmt_supported(struct fmts *fmts, gint fmts_num, guint fourcc)
{
	gint i;
	gboolean ret = FALSE;

	for (i = 0; i < fmts_num; i++) {
		if (fmts[i].fmt == fourcc) {
			ret = TRUE;
			break;
		}
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
set_fmt_ioctl_out(struct gst_backend_priv *priv, struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt;

	pix_fmt = &fmt->fmt.pix_mp;

	if (!is_pix_fmt_supported(priv->out_fmts, priv->out_fmts_num,
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

	priv->out_fourcc = pix_fmt->pixelformat;
	priv->out_buf_size = pix_fmt->plane_fmt[0].sizeimage;

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
set_fmt_ioctl_cap(struct gst_backend_priv *priv, struct v4l2_format *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt;

	pix_fmt = &fmt->fmt.pix_mp;

	if (!is_pix_fmt_supported(priv->cap_fmts, priv->cap_fmts_num,
				  pix_fmt->pixelformat)) {
		fprintf(stderr, "Unsupported pixelformat on CAPTURE\n");
		errno = EINVAL;
		return -1;
	}

	priv->cap_pix_fmt.pixelformat = pix_fmt->pixelformat;

	init_decoded_frame_params(pix_fmt);

	/* set unsupported parameters */
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = 0;
	pix_fmt->flags = 0;

	return 0;
}

int
set_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	g_mutex_lock(&priv->dev_lock);

	GST_OBJECT_LOCK(priv->pipeline);
	if (GST_STATE(priv->pipeline) != GST_STATE_NULL) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		GST_OBJECT_UNLOCK(priv->pipeline);
		g_mutex_unlock(&priv->dev_lock);
		return -1;
	}
	GST_OBJECT_UNLOCK(priv->pipeline);

	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = set_fmt_ioctl_out(priv, fmt);
	} else if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = set_fmt_ioctl_cap(priv, fmt);
	} else {
		fprintf(stderr, "Invalid buffer type\n");
		errno = EINVAL;
		ret = -1;
	}

	g_mutex_unlock(&priv->dev_lock);

	return ret;
}

static int
get_fmt_ioctl_cap(struct gst_backend_priv *priv,
		  struct v4l2_pix_format_mplane *pix_fmt)
{
	gint i;

	if (!priv->is_cap_fmt_acquirable) {
		errno = EINVAL;
		return -1;
	}

	DBG_LOG("cap format is acquirable\n");

	pix_fmt->width = priv->cap_pix_fmt.width;
	pix_fmt->height = priv->cap_pix_fmt.height;
	pix_fmt->pixelformat = priv->cap_pix_fmt.pixelformat;
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = 0;
	pix_fmt->flags = 0;
	pix_fmt->num_planes = priv->cap_pix_fmt.num_planes;

	DBG_LOG("width:%d height:%d num_plnaes=%d\n",
		pix_fmt->width, pix_fmt->height, pix_fmt->num_planes);

	if (priv->cap_pix_fmt.plane_fmt[0].sizeimage > 0) {
		for (i = 0; i < pix_fmt->num_planes; i++) {
			pix_fmt->plane_fmt[i].sizeimage =
					priv->
					cap_pix_fmt.plane_fmt[i].sizeimage;
			pix_fmt->plane_fmt[i].bytesperline =
					priv->
					cap_pix_fmt.plane_fmt[i].bytesperline;
		}
		pix_fmt->num_planes = priv->cap_pix_fmt.num_planes;
	} else {
		memset(pix_fmt->plane_fmt, 0, sizeof(pix_fmt->plane_fmt));
	}

	return 0;
}

int
get_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_format *fmt)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	struct v4l2_pix_format_mplane *pix_fmt;
	int ret;

	pix_fmt = &fmt->fmt.pix_mp;

	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		g_mutex_lock(&priv->dev_lock);
		pix_fmt->pixelformat = priv->out_fourcc;
		pix_fmt->plane_fmt[0].sizeimage = priv->out_buf_size;
		g_mutex_unlock(&priv->dev_lock);
		set_params_as_encoded_stream(pix_fmt);
		ret = 0;
	} else if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = get_fmt_ioctl_cap(priv, pix_fmt);
	} else {
		fprintf(stderr, "Invalid buffer type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

int
enum_fmt_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_fmtdesc *desc)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	struct fmts *fmts;
	gint fmts_num;

	if (!priv->out_fmts || !priv->cap_fmts) {
		fprintf(stderr, "Supported formats lists are not prepared\n");
		errno = EINVAL;
		return -1;
	}

	if (desc->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmts = priv->out_fmts;
		fmts_num = priv->out_fmts_num;
		desc->flags = V4L2_FMT_FLAG_COMPRESSED;
	} else if (desc->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmts = priv->cap_fmts;
		fmts_num = priv->cap_fmts_num;
		desc->flags = 0;
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		return -1;
	}

	if (fmts_num <= desc->index) {
		errno = EINVAL;
		return -1;
	}

	desc->pixelformat = fmts[desc->index].fmt;
	g_strlcpy((gchar *)desc->description, fmts[desc->index].fmt_char,
		   sizeof(desc->description));
	memset(desc->reserved, 0, sizeof(desc->reserved));

	return 0;
}
int
enum_framesizes_ioctl (struct v4l_gst_priv *dev_ops_priv, struct v4l2_frmsizeenum *argp) {
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;

	switch (argp->pixel_format) {
        case V4L2_PIX_FMT_GREY:
        case V4L2_PIX_FMT_RGB565:
        case V4L2_PIX_FMT_RGB24:
        case V4L2_PIX_FMT_BGR24:
        case V4L2_PIX_FMT_ABGR32:
        case V4L2_PIX_FMT_XBGR32:
        case V4L2_PIX_FMT_ARGB32:
        case V4L2_PIX_FMT_XRGB32:
        case V4L2_PIX_FMT_RGB32:
        case V4L2_PIX_FMT_BGR32:
        case V4L2_PIX_FMT_H264:
		argp->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
		argp->stepwise.step_width = 1;
		argp->stepwise.step_height = 1;
		break;
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
        case V4L2_PIX_FMT_NV12MT:
		argp->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		argp->stepwise.step_width = 2;
		argp->stepwise.step_height = 2;
		break;
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_YVYU:
        case V4L2_PIX_FMT_YUV422P:
		argp->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		argp->stepwise.step_width = 2;
		argp->stepwise.step_height = 1;
		break;
        case V4L2_PIX_FMT_YVU410:
        case V4L2_PIX_FMT_YUV410:
		argp->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		argp->stepwise.step_width = 4;
		argp->stepwise.step_height = 4;
		break;
        case V4L2_PIX_FMT_YUV411P:
		argp->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		argp->stepwise.step_width = 4;
		argp->stepwise.step_height = 1;
		break;
	}
	argp->stepwise.min_width = 16;
	argp->stepwise.min_height = 16;
	argp->stepwise.max_width = priv->max_width ?
		priv->max_width : 1920;
	argp->stepwise.max_height = priv->max_height ?
		priv->max_height : 1088;

	return 0;
}

int
get_ctrl_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_control *ctrl)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->value = priv->cap_min_buffers;
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

	if (!buffers) {
		fprintf(stderr, "Buffers list is not set\n");
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
	struct v4l_gst_buffer *buffer = data;
	struct gst_backend_priv *priv;

	priv = buffer->priv;

	release_out_buffer(priv, buffer->buffer);
}

static int
qbuf_ioctl_out(struct gst_backend_priv *priv, struct v4l2_buffer *buf)
{
	GstFlowReturn flow_ret;
	GstBuffer *wrapped_buffer;
	GstMapInfo info;
	struct v4l_gst_buffer *buffer;

	if (!check_v4l2_buffer(buf, priv->out_buffers, priv->out_buffers_num,
			       priv->src_pool))
		return -1;

	buffer = &priv->out_buffers[buf->index];

	if (buf->m.planes[0].bytesused == 0) {
		flow_ret = gst_app_src_end_of_stream(GST_APP_SRC(priv->appsrc));
		if (flow_ret != GST_FLOW_OK) {
			fprintf(stderr, "Failed to send an eos event\n");
			errno = EINVAL;
			return -1;
		}

		gst_buffer_unmap(buffer->buffer, &buffer->info);
		memset(&buffer->info, 0, sizeof(buffer->info));

		buffer->state = V4L_GST_BUFFER_QUEUED;
		priv->eos_buffer = buffer->buffer;

		return 0;
	}

	if (buffer->state == V4L_GST_BUFFER_QUEUED) {
		fprintf(stderr, "Invalid buffer state\n");
		errno = EINVAL;
		return -1;
	}

	DBG_LOG("queue index=%d buffer=%p\n", buf->index,
		priv->out_buffers[buf->index].buffer);

	gst_buffer_unmap(buffer->buffer, &buffer->info);
	memset(&buffer->info, 0, sizeof(buffer->info));

	/* Rewrap an input buffer with the just size of bytesused
	   because it will be regarded as having data filled to the entire
	   buffer size internally in the GStreame pipeline.
	   Also set the destructor (notify_unref()). */

	if (!gst_buffer_map(buffer->buffer, &info, GST_MAP_READ)) {
		fprintf(stderr, "Failed to map buffer (%p)\n", buffer->buffer);
		errno = EINVAL;
		return -1;
	}

	wrapped_buffer = gst_buffer_new_wrapped_full(
					GST_MEMORY_FLAG_READONLY, info.data,
					buf->m.planes[0].bytesused, 0,
					buf->m.planes[0].bytesused,
					buffer, notify_unref);

	gst_buffer_unmap(buffer->buffer, &info);

	DBG_LOG("buffer rewrap ts=%ld\n", buf->timestamp.tv_sec);
	GST_BUFFER_PTS(wrapped_buffer) = GST_TIMEVAL_TO_TIME(buf->timestamp);

	buffer->state = V4L_GST_BUFFER_QUEUED;

	flow_ret = gst_app_src_push_buffer(
			GST_APP_SRC(priv->appsrc), wrapped_buffer);
	if (flow_ret != GST_FLOW_OK) {
		fprintf(stderr,
			"Failed to push a buffer to the pipeline on OUTPUT"
			"(index=%d)\n", buf->index);
		errno = EINVAL;
		return -1;
	}

	return 0;
}

static int
qbuf_ioctl_cap(struct gst_backend_priv *priv, struct v4l2_buffer *buf)
{
	struct v4l_gst_buffer *buffer;

	if (!check_v4l2_buffer(buf, priv->cap_buffers, priv->cap_buffers_num,
			       priv->sink_pool))
		return -1;

	buffer = &priv->cap_buffers[buf->index];

	if (buffer->state == V4L_GST_BUFFER_QUEUED) {
		fprintf(stderr, "Invalid buffer state\n");
		errno = EINVAL;
		return -1;
	}

	gst_buffer_unmap(buffer->buffer, &buffer->info);
	memset(&buffer->info, 0, sizeof(buffer->info));

	if (buffer->state == V4L_GST_BUFFER_READY_FOR_DEQUEUE) {
		buffer->state = V4L_GST_BUFFER_QUEUED;
		g_queue_push_tail(priv->cap_buffers_queue, buffer->buffer);
		g_mutex_lock(&priv->queue_mutex);
		g_cond_signal(&priv->queue_cond);
		g_mutex_unlock(&priv->queue_mutex);
		return 0;
	}

	if (buffer->state == V4L_GST_BUFFER_DEQUEUED) {
		DBG_LOG("unref buffer: %p, index=%d\n", buffer->buffer, buf->index);
		gst_buffer_unref(buffer->buffer);
	}

	buffer->state = V4L_GST_BUFFER_QUEUED;

	return 0;
}

int
qbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	g_mutex_lock(&priv->dev_lock);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = qbuf_ioctl_out(priv, buf);
		if (ret < 0) {
			g_mutex_unlock(&priv->dev_lock);
			return ret;
		}
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = qbuf_ioctl_cap(priv, buf);
		if (ret < 0) {
			g_mutex_unlock(&priv->dev_lock);
			return 0;
		}
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		g_mutex_unlock(&priv->dev_lock);
		return -1;
	}

	g_mutex_unlock(&priv->dev_lock);

	return ret;
}

static inline guint
calc_plane_size(GstVideoInfo *info, GstVideoMeta *meta, gint index)
{
	return meta->stride[index] * GST_VIDEO_INFO_COMP_HEIGHT(info, index);
}

static void
set_v4l2_buffer_plane_params(struct gst_backend_priv *priv,
			     struct v4l_gst_buffer *buffers, guint n_planes,
			     guint bytesused[], struct timeval *timestamp,
			     struct v4l2_buffer *buf)
{
	gint i;

	memcpy(buf->m.planes, buffers[buf->index].planes,
	       sizeof(struct v4l2_plane) * n_planes);

	if (bytesused) {
		for (i = 0; i < n_planes; i++)
			buf->m.planes[i].bytesused = bytesused[i];
	}

	if (timestamp) {
		buf->timestamp.tv_sec = timestamp->tv_sec;
		buf->timestamp.tv_usec = timestamp->tv_usec;
	} else {
		buf->timestamp.tv_sec = buf->timestamp.tv_usec = 0;
	}
}

static int
fill_v4l2_buffer(struct gst_backend_priv *priv, GstBufferPool *pool,
		 struct v4l_gst_buffer *buffers, gint buffers_num,
		 guint bytesused[], struct timeval *timestamp,
		 struct v4l2_buffer *buf)
{
	GstVideoMeta *meta = NULL;
	guint n_planes;

	get_raw_video_params(pool, buffers[buf->index].buffer, NULL, &meta);

	n_planes = (meta) ? meta->n_planes : 1;

	set_v4l2_buffer_plane_params(priv, buffers, n_planes, bytesused,
				     timestamp, buf);

	/* set unused params */
	memset(&buf->timecode, 0, sizeof(buf->timecode));
	buf->sequence = 0;
	buf->flags = 0;
	buf->field = V4L2_FIELD_NONE;

	buf->length = n_planes;

	return 0;
}


static GstBuffer *
dequeue_blocking(struct gst_backend_priv *priv, GQueue *queue, GCond *cond)
{
	GstBuffer *buffer;

	buffer = g_queue_pop_head(queue);
	while (!buffer && priv->is_pipeline_started) {
		g_cond_wait(cond, &priv->queue_mutex);
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
dequeue_buffer(struct gst_backend_priv *priv, GQueue *queue, GCond *cond,
		int type)
{
	GstBuffer *buffer = NULL;

	g_mutex_lock(&priv->queue_mutex);

	if (priv->dev_ops_priv->is_non_blocking)
		buffer = dequeue_non_blocking(queue);
	else
		buffer = dequeue_blocking(priv, queue, cond);

	if (buffer) {
		if (g_queue_is_empty(priv->cap_buffers_queue)) {
			clear_event(priv->dev_ops_priv->event_state, POLLOUT);
		}
	}

	g_mutex_unlock(&priv->queue_mutex);

	return buffer;
}

static GstBuffer *
acquire_buffer_from_pool(struct gst_backend_priv *priv, GstBufferPool *pool)
{
	GstFlowReturn flow_ret;
	GstBuffer *buffer;
	GstBufferPoolAcquireParams params = { 0, };

	if (priv->dev_ops_priv->is_non_blocking)
		params.flags |= GST_BUFFER_POOL_ACQUIRE_FLAG_DONTWAIT;

	flow_ret = gst_buffer_pool_acquire_buffer(pool, &buffer, &params);

	if (priv->dev_ops_priv->is_non_blocking && flow_ret == GST_FLOW_EOS) {
		DBG_LOG("The buffer pool is empty in "
			"the non-blocking mode, return EAGAIN\n");
		errno = EAGAIN;
		return NULL;
	} else if (flow_ret != GST_FLOW_OK) {
		fprintf(stderr, "gst_buffer_pool_acquire_buffer failed\n");
		errno = EINVAL;
		return NULL;
	}

	if (g_atomic_int_dec_and_test(&priv->empty_out_buffer_cnt))
		clear_event(priv->dev_ops_priv->event_state, POLLIN);

	return buffer;
}

static int
dqbuf_ioctl_out(struct gst_backend_priv *priv, struct v4l2_buffer *buf)
{
	GstBuffer *buffer;
	guint index;

	if (!priv->is_pipeline_started) {
		fprintf(stderr, "The pipeline does not start yet.\n");
		errno = EINVAL;
		return -1;
	}

	if (!check_no_index_v4l2_buffer(buf, priv->out_buffers,
					priv->src_pool))
		return -1;

	buffer = acquire_buffer_from_pool(priv, priv->src_pool);

#ifdef ENABLE_CHROMIUM_COMPAT
	if (!priv->is_cap_fmt_acquirable) {
		if (!priv->pending_buffer) {
			DBG_LOG("Holding the first acquired buffer "
				"on OUTPUT\n");
			priv->pending_buffer = buffer;
			errno = EAGAIN;
			return -1;
		}
	} else {
		if (!buffer && priv->pending_buffer) {
			buffer = priv->pending_buffer;
			priv->pending_buffer = NULL;
		}
	}
#endif

	if (!buffer)
		return -1;

	index = get_v4l2_buffer_index(priv->out_buffers,
				      priv->out_buffers_num, buffer);
	if (index >= priv->out_buffers_num) {
		fprintf(stderr, "Failed to get a valid buffer index "
			"on OUTPUT\n");
		errno = EINVAL;
		return -1;
	}

	buf->index = index;
	priv->out_buffers[buf->index].state = V4L_GST_BUFFER_DEQUEUED;

	DBG_LOG("success dequeue buffer index=%d buffer=%p\n", index, buffer);

	return fill_v4l2_buffer(priv, priv->src_pool,
				priv->out_buffers, priv->out_buffers_num,
				NULL, NULL, buf);
}

static int
dqbuf_ioctl_cap(struct gst_backend_priv *priv, struct v4l2_buffer *buf)
{
	GstBuffer *buffer;
	guint index;
	struct timeval timestamp;
	guint bytesused[GST_VIDEO_MAX_PLANES];
	gint i;

	if (!check_no_index_v4l2_buffer(buf, priv->cap_buffers,
					priv->sink_pool))
		return -1;

	buffer = dequeue_buffer(priv, priv->cap_buffers_queue,
				&priv->queue_cond, buf->type);
	if (!buffer)
		return -1;

	index = get_v4l2_buffer_index(priv->cap_buffers,
				      priv->cap_buffers_num, buffer);
	if (index >= priv->cap_buffers_num) {
		fprintf(stderr, "Failed to get a valid buffer index "
			"on CAPTURE\n");
		errno = EINVAL;
		gst_buffer_unref(buffer);
		return -1;
	}

	buf->index = index;

	for (i = 0; i < priv->cap_pix_fmt.num_planes; i++)
		bytesused[i] = priv->cap_pix_fmt.plane_fmt[i].sizeimage;

	GST_TIME_TO_TIMEVAL(GST_BUFFER_PTS(buffer), timestamp);

	priv->cap_buffers[buf->index].state = V4L_GST_BUFFER_DEQUEUED;

	DBG_LOG("success dequeue buffer index=%d buffer=%p ts=%ld\n",
		index, buffer, timestamp.tv_sec);

	return fill_v4l2_buffer(priv, priv->sink_pool,
				priv->cap_buffers, priv->cap_buffers_num,
				bytesused, &timestamp, buf);
}

int
dqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	g_mutex_lock(&priv->dev_lock);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = dqbuf_ioctl_out(priv, buf);
		if (ret < 0) {
			g_mutex_unlock(&priv->dev_lock);
			return ret;
		}

	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = dqbuf_ioctl_cap(priv, buf);
		if (ret < 0) {
			g_mutex_unlock(&priv->dev_lock);
			return ret;
		}

	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		g_mutex_unlock(&priv->dev_lock);
		return -1;
	}

	g_mutex_unlock(&priv->dev_lock);

	return ret;
}

int
querybuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_buffer *buf)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	struct v4l_gst_buffer *buffers;
	gint buffers_num;
	GstBufferPool *pool;
	int ret;

	g_mutex_lock(&priv->dev_lock);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		buffers = priv->out_buffers;
		buffers_num = priv->out_buffers_num;
		pool = priv->src_pool;
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		buffers = priv->cap_buffers;
		buffers_num = priv->cap_buffers_num;
		pool = priv->sink_pool;
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		g_mutex_unlock(&priv->dev_lock);
		return -1;
	}

	if (!check_v4l2_buffer(buf, buffers, buffers_num, pool)) {
		g_mutex_unlock(&priv->dev_lock);
		return -1;
	}

	ret = fill_v4l2_buffer(priv, pool, buffers, buffers_num,
			       NULL, NULL, buf);

	g_mutex_unlock(&priv->dev_lock);

	return ret;
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

	if (g_strcmp0(mime, GST_VIDEO_CODEC_MIME_H264) == 0) {
		return gst_caps_new_simple(mime, "stream-format",
					   G_TYPE_STRING, "byte-stream", NULL);
	}

	return gst_caps_new_empty_simple(mime);
}

#define PAGE_ALIGN(off, align) ((off + align - 1) & ~(align - 1))

static gsize
set_mem_offset(struct v4l_gst_buffer *buffer, GstBufferPool *pool, gsize offset)
{
	GstVideoInfo info;
	GstVideoMeta *meta;
	static long page_size = -1;
	gint i;

	if (page_size < 0)
		page_size = sysconf(_SC_PAGESIZE);

	if (!get_raw_video_params(pool, buffer->buffer, &info, &meta)) {
		/* deal with this as a single plane */
		buffer->planes[0].m.mem_offset = offset;
		return PAGE_ALIGN(gst_buffer_get_size(buffer->buffer),
			page_size) + offset;
	}

	if (meta) {
		for (i = 0; i < meta->n_planes; i++) {
			buffer->planes[i].m.mem_offset = offset;
			offset += PAGE_ALIGN(calc_plane_size(&info, meta, i),
				page_size);
		}
	}

	return offset;
}

static guint
alloc_buffers_from_pool(struct gst_backend_priv *priv, GstBufferPool *pool,
			struct v4l_gst_buffer **buffers)
{
	GstBufferPoolAcquireParams params = { 0, };
	GstFlowReturn flow_ret;
	guint actual_max_buffers;
	struct v4l_gst_buffer *bufs_list;
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

		bufs_list[i].priv = priv;
		bufs_list[i].state = V4L_GST_BUFFER_DEQUEUED;

		DBG_LOG("out gst_buffer[%d] : %p\n", i, bufs_list[i].buffer);
	}

	*buffers = bufs_list;

	DBG_LOG("The number of buffers actually set to the buffer pool is %d\n",
		actual_max_buffers);

	return actual_max_buffers;

	/* error cases */
free_bufs_list:
	for (i = 0; i < actual_max_buffers; i++) {
		if (bufs_list[i].buffer)
			gst_buffer_unref(bufs_list[i].buffer);
	}
	g_free(bufs_list);
inactivate_pool:
	gst_buffer_pool_set_active(pool, FALSE);

	return 0;
}

static int
force_dqbuf_from_pool(GstBufferPool *pool, struct v4l_gst_buffer *buffers,
		      gint buffers_num, gboolean map)
{
	GstFlowReturn flow_ret;
	GstBufferPoolAcquireParams params = { 0, };
	GstBuffer *buffer;
	guint index;

	params.flags = GST_BUFFER_POOL_ACQUIRE_FLAG_DONTWAIT;

	/* force to make buffers available to the V4L2 caller side */
	flow_ret = gst_buffer_pool_acquire_buffer(pool, &buffer, &params);
	if (flow_ret != GST_FLOW_OK)
		return -1;

	index = get_v4l2_buffer_index(buffers, buffers_num, buffer);
	if (index >= buffers_num) {
		fprintf(stderr, "Failed to get a valid buffer index\n");
		errno = EINVAL;
		return -1;
	}

	if (!map)
		return index;

	if (!gst_buffer_map(buffer, &buffers[index].info,
			    buffers[index].flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n", buffer);
		errno = EINVAL;
		return -1;
	}
	return index;
}

static int
force_out_dqbuf(struct gst_backend_priv *priv)
{
	int index;
	while ((index = force_dqbuf_from_pool(priv->src_pool, priv->out_buffers,
			 priv->out_buffers_num, true)) >= 0) {
		priv->out_buffers[index].state = V4L_GST_BUFFER_DEQUEUED;
		g_atomic_int_dec_and_test(&priv->empty_out_buffer_cnt);
	}

	clear_event(priv->dev_ops_priv->event_state, POLLIN);

	DBG_LOG("empty_out_buffer_cnt : %d\n", priv->empty_out_buffer_cnt);

	return 0;
}

static int
force_cap_dqbuf(struct gst_backend_priv *priv)
{
	GstBuffer *buffer;
	gint index;
	gint recommit_index = -1;

	g_mutex_lock(&priv->queue_mutex);

	clear_event(priv->dev_ops_priv->event_state, POLLOUT);

	while ((index = force_dqbuf_from_pool(priv->sink_pool, priv->cap_buffers,
			 priv->cap_buffers_num, false)) >= 0) {
		if (priv->cap_buffers[index].state == V4L_GST_BUFFER_PREROLLED) {
			priv->cap_buffers[index].state = V4L_GST_BUFFER_INIT;
			recommit_index = index;
		} else {
			priv->cap_buffers[index].state = V4L_GST_BUFFER_DEQUEUED;
		}
	}

	if (recommit_index >= 0)
		gst_buffer_unref(priv->cap_buffers[recommit_index].buffer);

	while ((buffer = g_queue_pop_head(priv->cap_buffers_queue))) {
		index = get_v4l2_buffer_index(priv->cap_buffers, priv->cap_buffers_num, buffer);
		if (index < priv->cap_buffers_num)
			priv->cap_buffers[index].state = V4L_GST_BUFFER_DEQUEUED;
	}

	g_mutex_unlock(&priv->queue_mutex);

	return 0;
}

static int
flush_pipeline(struct gst_backend_priv *priv)
{
	GstEvent *event;

	DBG_LOG("flush start\n");

	gst_buffer_pool_set_flushing(priv->src_pool, true);
	gst_buffer_pool_set_flushing(priv->sink_pool, true);

	event = gst_event_new_flush_start();
	if (!gst_element_send_event(priv->pipeline, event)) {
		fprintf(stderr, "Failed to send a flush start event\n");
		errno = EINVAL;
		return -1;
	}

	event = gst_event_new_flush_stop(TRUE);
	if (!gst_element_send_event(priv->pipeline, event)) {
		fprintf(stderr, "Failed to send a flush stop event\n");
		errno = EINVAL;
		return -1;
	}

	gst_buffer_pool_set_flushing(priv->src_pool, false);
	gst_buffer_pool_set_flushing(priv->sink_pool, false);

	DBG_LOG("flush end\n");

	return 0;
}

static int
streamoff_ioctl_out(struct gst_backend_priv *priv, gboolean steal_ref)
{
	int ret;

	GST_OBJECT_LOCK(priv->pipeline);
	if (GST_STATE(priv->pipeline) == GST_STATE_NULL) {
		/* No need to flush the pipeline after it has been
		   the NULL state. */
		GST_OBJECT_UNLOCK(priv->pipeline);
		goto flush_buffer_queues;
	}
	GST_OBJECT_UNLOCK(priv->pipeline);


	ret = flush_pipeline(priv);

	if (ret < 0)
		return ret;

flush_buffer_queues:
	/* Vacate the buffers queues to make them available in the next time */
	ret = force_out_dqbuf(priv);
	if (ret < 0)
		return ret;
	ret = force_cap_dqbuf(priv);
	if (ret < 0)
		return ret;

	/* The reference counted up below will be unreffed when calling
	   the streamon ioctl. This prevents from returning all the buffers
	   of the OUTPUT bufferpool and freeing them by inactivating
	   the bufferpool for flushing. */
	if (steal_ref)
		gst_buffer_ref(priv->out_buffers[0].buffer);

	/* wake up blocking of the OUTPUT buffers acquistion */
	if (!gst_buffer_pool_set_active(priv->src_pool, FALSE)) {
		fprintf(stderr, "Failed to inactivate buffer pool on OUTPUT\n");
		errno = EINVAL;
		return -1;
	}

	/* wake up blocking of the CAPTURE buffers acquistion */
	g_mutex_lock(&priv->queue_mutex);
	priv->is_pipeline_started = FALSE;
	g_cond_broadcast(&priv->queue_cond);
	g_mutex_unlock(&priv->queue_mutex);

	return 0;
}

static int
reqbuf_ioctl_out(struct gst_backend_priv *priv,
		 struct v4l2_requestbuffers *req)
{
	GstCaps *caps;
	guint adjusted_count;
	guint allocated_num;
	int ret;
	guint i;

	if (!is_supported_memory_io(req->memory)) {
		fprintf(stderr, "Only V4L2_MEMORY_MMAP is supported\n");
		return -1;
	}

	g_mutex_lock(&priv->dev_lock);

	if (req->count == 0) {
		/* The following function flushes both the OUTPUT and CAPTURE
		   buffer types because the GStreamer can only flush the whole
		   of the pipeline, so the buffers of both the buffer types
		   need to be requeued after this operation.
		*/
		ret = streamoff_ioctl_out(priv, FALSE);
		if (ret < 0)
			goto unlock;

		/* Force to return dequeued buffers to the buffer pool. */
		for (i = 0; i < priv->out_buffers_num; i++) {
			if (priv->out_buffers[i].state ==
			    V4L_GST_BUFFER_DEQUEUED) {
				gst_buffer_unref(priv->out_buffers[i].buffer);
			}
		}

		if (priv->out_buffers) {
			g_free(priv->out_buffers);
			priv->out_buffers = NULL;
		}

		ret = 0;
		goto unlock;
	}

	if (priv->is_pipeline_started) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		ret = -1;
		goto unlock;
	}

	if (gst_buffer_pool_is_active(priv->src_pool)) {
		if (!gst_buffer_pool_set_active(priv->src_pool, FALSE)) {
			fprintf(stderr, "Failed to inactivate buffer pool\n");
			errno = EBUSY;
			ret = -1;
			goto unlock;
		}
	}

	caps = get_codec_caps_from_fourcc(priv->out_fourcc);
	if (!caps) {
		errno = EINVAL;
		ret = -1;
		goto unlock;
	}

	adjusted_count = MIN(req->count, VIDEO_MAX_FRAME);

	set_buffer_pool_params(priv->src_pool, caps,
			       priv->out_buf_size, adjusted_count,
			       adjusted_count);

	allocated_num = alloc_buffers_from_pool(priv, priv->src_pool,
						&priv->out_buffers);
	if (allocated_num == 0) {
		gst_caps_unref(caps);
		ret = -1;
		goto unlock;
	}

	for (i = 0; i < allocated_num; i++) {
		/* Set identifiers for associating a GstBuffer with
		   a V4L2 buffer in the V4L2 caller side. */
		priv->mmap_offset =
			set_mem_offset(&priv->out_buffers[i],
			priv->src_pool,
			priv->mmap_offset);

		priv->out_buffers[i].planes[0].length =
				gst_buffer_get_size(priv->out_buffers[i].buffer);
	}

	req->count = priv->out_buffers_num = allocated_num;

	DBG_LOG("buffers count=%d\n", req->count);

	g_atomic_int_set(&priv->empty_out_buffer_cnt, 0);

	ret = 0;

unlock:
	g_mutex_unlock(&priv->dev_lock);

	return ret;
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

static void
update_cap_buffer_count(struct gst_backend_priv *priv)
{
	g_mutex_unlock(&priv->dev_lock);

	/* Wait for the first buffer to come out of the pipeline to
           check the bufferpool properties */
	g_mutex_lock(&priv->queue_mutex);
	while (priv->cap_buffers_num == 0)
		g_cond_wait(&priv->queue_cond, &priv->queue_mutex);
	g_mutex_unlock(&priv->queue_mutex);

	g_mutex_lock(&priv->dev_lock);
	DBG_LOG("The number of buffers actually set to the buffer pool is %d\n",
		priv->cap_buffers_num);
}

static int
reqbuf_ioctl_cap(struct gst_backend_priv *priv,
		 struct v4l2_requestbuffers *req)
{
	GstStateChangeReturn state_ret;
	int ret;
	gint i;

	if (!is_supported_memory_io(req->memory)) {
		fprintf(stderr, "Only V4L2_MEMORY_MMAP is supported\n");
		return -1;
	}

	g_mutex_lock(&priv->dev_lock);

	if (req->count == 0) {
		state_ret = gst_element_set_state(priv->pipeline,
						  GST_STATE_NULL);
		while (state_ret == GST_STATE_CHANGE_ASYNC) {
			/* This API blocks up to the ASYNC state change completion. */
			g_mutex_unlock(&priv->dev_lock);
			state_ret = gst_element_get_state(priv->pipeline, NULL,
							  NULL,
							  GST_CLOCK_TIME_NONE);
			g_mutex_lock(&priv->dev_lock);
		}

		if (state_ret != GST_STATE_CHANGE_SUCCESS) {
			fprintf(stderr, "Failed to stop pipeline (ret:%s)\n",
				gst_element_state_change_return_get_name(
								state_ret));
			errno = EINVAL;
			ret = -1;
			goto unlock;
		}

		priv->is_cap_fmt_acquirable = FALSE;

		for (i = 0; i < priv->cap_buffers_num - 1; i++) {
			if (priv->cap_buffers[i].state ==
			    V4L_GST_BUFFER_DEQUEUED) {
				gst_buffer_unref(priv->cap_buffers[i].buffer);
			}
		}

		g_queue_clear(priv->cap_buffers_queue);

		g_mutex_lock(&priv->queue_mutex);
		priv->is_pipeline_started = FALSE;
		g_cond_broadcast(&priv->queue_cond);
		g_mutex_unlock(&priv->queue_mutex);

		if (priv->cap_buffers) {
			g_free(priv->cap_buffers);
			priv->cap_buffers = NULL;
		}
		priv->cap_buffers_num = 0;
		init_decoded_frame_params(&priv->cap_pix_fmt);

		ret = 0;
		goto unlock;
	}

	if (!priv->is_pipeline_started) {
		fprintf(stderr,
			"Need to start the pipeline for the buffer request "
			"on CAPTURE\n");
		errno = EINVAL;
		ret = -1;
		goto unlock;
	}
	if (priv->cap_buffers_num) {
		req->count = priv->cap_buffers_num - 1;
		ret = 0;
		goto unlock;
	}

	g_mutex_lock(&priv->cap_reqbuf_mutex);

	priv->cap_buffers_req = MIN(req->count, VIDEO_MAX_FRAME);
	priv->cap_buffers_req++; /*An extra buffer to keep the preroll*/

	g_cond_signal(&priv->cap_reqbuf_cond);
	g_mutex_unlock(&priv->cap_reqbuf_mutex);

	update_cap_buffer_count(priv);

	priv->cap_buffers_req = 0;

	if (priv->cap_buffers_num == 0) {
		ret = -1;
		goto unlock;
	}

	req->count = priv->cap_buffers_num - 1; /* But don't tell the application about our secret buffer */

	DBG_LOG("buffers count=%d\n", req->count);

	ret = 0;

unlock:
	g_mutex_unlock(&priv->dev_lock);

	return ret;
}

int
reqbuf_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_requestbuffers *req)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	if (req->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = reqbuf_ioctl_out(priv, req);
	} else if (req->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = reqbuf_ioctl_cap(priv, req);
	} else {
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
set_out_format_to_pipeline(struct gst_backend_priv *priv)
{
	GstCaps *caps;

	caps = get_codec_caps_from_fourcc(priv->out_fourcc);
	if (!caps) {
		errno = EINVAL;
		return FALSE;
	}

	gst_app_src_set_caps(GST_APP_SRC(priv->appsrc), caps);
	gst_caps_unref(caps);

	return TRUE;
}

static gboolean
set_cap_format_to_pipeline(struct gst_backend_priv *priv)
{
	GstElement *peer_elem;
	GstCaps *caps;
	GstVideoFormat fmt;
	gboolean ret;

	fmt = convert_video_format_v4l2_to_gst(priv->
					       cap_pix_fmt.pixelformat);
	if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
		fprintf(stderr, "Invalid format on CAPTURE\n");
		errno = EINVAL;
		return FALSE;
	}

	caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
				   gst_video_format_to_string(fmt), NULL);

	peer_elem = get_peer_element(priv->appsink, "sink");
	if (!relink_elements_with_caps_filtered(peer_elem, priv->appsink,
						caps)) {
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

static int
streamon_ioctl_out(struct gst_backend_priv *priv)
{
	GstState state;

	if (priv->is_pipeline_started) {
		fprintf(stderr, "The pipeline is already running\n");
		errno = EBUSY;
		return -1;
	}

	GST_OBJECT_LOCK(priv->pipeline);
	state = GST_STATE(priv->pipeline);
	GST_OBJECT_UNLOCK(priv->pipeline);

	g_mutex_lock(&priv->dev_lock);

	if (state == GST_STATE_NULL) {
		if (!set_out_format_to_pipeline(priv))
			return -1;
		if (!set_cap_format_to_pipeline(priv))
			return -1;
	}

	if (!gst_buffer_pool_is_active(priv->src_pool)) {
		if (!gst_buffer_pool_set_active(priv->src_pool, TRUE)) {
			fprintf(stderr, "Failed to activate buffer pool\n");
			errno = EINVAL;
			return -1;
		}

		/* Restore the extra reference counted up in the streamoff */
		gst_buffer_unref(priv->out_buffers[0].buffer);
	}

	gst_element_set_state(priv->pipeline, GST_STATE_PLAYING);

	priv->is_pipeline_started = TRUE;

	g_mutex_unlock(&priv->dev_lock);

	return 0;
}

int
streamon_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	if (*type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		DBG_LOG("streamon on OUTPUT\n");
		ret = streamon_ioctl_out(priv);
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

int
streamoff_ioctl(struct v4l_gst_priv *dev_ops_priv, enum v4l2_buf_type *type)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int ret;

	if (*type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		g_mutex_lock(&priv->dev_lock);
		ret = streamoff_ioctl_out(priv, TRUE);
		g_mutex_unlock(&priv->dev_lock);
	} else if (*type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* no processing */
		ret = 0;
	} else {
		fprintf(stderr, "Invalid buf type\n");
		errno = EINVAL;
		ret = -1;
	}

	return ret;
}

int
subscribe_event_ioctl(struct v4l_gst_priv *dev_ops_priv,
		      struct v4l2_event_subscription *sub)
{
	return 0;
}

int
dqevent_ioctl(struct v4l_gst_priv *dev_ops_priv, struct v4l2_event *ev)
{
	/* TODO: Add the implementation for subscribed event notifications.
		 Always return failure until the feature has been supported. */
	return -1;
}

static int
find_out_buffer_by_offset(struct v4l_gst_priv *dev_ops_priv, int64_t offset)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	gint index = -1;
	gint i;

	for (i = 0; i < priv->out_buffers_num; i++) {
		if (priv->out_buffers[i].planes[0].m.mem_offset == offset) {
			index = i;
			break;
		}
	}

	return index;
}

static void *
map_out_buffer(struct v4l_gst_priv *dev_ops_priv, int index, int prot)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	GstMapInfo info;
	void *data;
	GstMapFlags map_flags;

	map_flags = (prot & PROT_READ) ? GST_MAP_READ : 0;
	map_flags |= (prot & PROT_WRITE) ? GST_MAP_WRITE : 0;

	if (!gst_buffer_map(priv->out_buffers[index].buffer, &info,
			    map_flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n",
			priv->out_buffers[index].buffer);
		errno = EINVAL;
		return MAP_FAILED;
	}

	data = info.data;

	gst_buffer_unmap(priv->out_buffers[index].buffer, &info);

	priv->out_buffers[index].flags = map_flags;

	return data;
}

static int
find_cap_buffer_by_offset(struct v4l_gst_priv *dev_ops_priv, int64_t offset,
			  int *index, int *plane)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	gint i, j;

	for (i = 0; i < priv->cap_buffers_num; i++) {
		for (j = 0; j < priv->cap_pix_fmt.num_planes; j++) {
			if (priv->cap_buffers[i].planes[j].m.mem_offset ==
			    offset) {
				*index = i;
				*plane = j;
				return 0;
			}
		}
	}

	return -1;
}

static void *
map_cap_buffer(struct v4l_gst_priv *dev_ops_priv, int index, int plane,
	       int prot)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	GstVideoMeta *meta;
	GstMapInfo info;
	void *data;
	GstMapFlags map_flags;

	map_flags = (prot & PROT_READ) ? GST_MAP_READ : 0;
	map_flags |= (prot & PROT_WRITE) ? GST_MAP_WRITE : 0;

	if (!gst_buffer_map(priv->cap_buffers[index].buffer, &info,
			    map_flags)) {
		fprintf(stderr, "Failed to map buffer (%p)\n",
			priv->cap_buffers[index].buffer);
		errno = EINVAL;
		return MAP_FAILED;
	}

	if (!get_raw_video_params(priv->sink_pool,
				  priv->cap_buffers[index].buffer,
				  NULL, &meta)) {
		fprintf(stderr, "Failed to get video meta data\n");
		errno = EINVAL;
		gst_buffer_unmap(priv->cap_buffers[index].buffer,
				 &priv->cap_buffers[index].info);
		return MAP_FAILED;
	}

	data = info.data + meta->offset[plane];

	gst_buffer_unmap(priv->cap_buffers[index].buffer, &info);

	priv->cap_buffers[index].flags = map_flags;

	return data;
}

void *
gst_backend_mmap(struct v4l_gst_priv *dev_ops_priv, void *start, size_t length,
		 int prot, int flags, int fd, int64_t offset)
{
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	int index;
	int plane;
	void *map = MAP_FAILED;
	int ret;

	/* unused */
	(void)start;
	(void)flags;
	(void)fd;

	/* The GStreamer memory mapping internally maps
	   the whole allocated size of a buffer, so the mapping length
	   does not need to be specified. */
	(void)length;

	g_mutex_lock(&priv->dev_lock);

	index = find_out_buffer_by_offset(dev_ops_priv, offset);
	if (index >= 0) {
		map = map_out_buffer(dev_ops_priv, index, prot);
		goto unlock;
	}

	ret = find_cap_buffer_by_offset(dev_ops_priv, offset, &index, &plane);
	if (ret == 0) {
		map = map_cap_buffer(dev_ops_priv, index, plane, prot);
		goto unlock;
	}

unlock:
	g_mutex_unlock(&priv->dev_lock);

	DBG_LOG("Final map = %x\n", map);

	return map;
}
int expbuf_ioctl(struct v4l_gst_priv *dev_ops_priv,
		 struct v4l2_exportbuffer *expbuf) {
	struct v4l_gst_buffer *buffer;
	struct gst_backend_priv *priv = dev_ops_priv->gst_priv;
	guint plane = 0;
	guint i;
	if (expbuf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fprintf(stderr, "Can only export capture buffers as dmebuf\n");
		errno = EINVAL;
		return -1;
	}

	g_mutex_lock(&priv->queue_mutex);
	buffer = &priv->cap_buffers[expbuf->index];
	while (buffer->state != V4L_GST_BUFFER_READY_FOR_DEQUEUE)
		g_cond_wait(&priv->queue_cond, &priv->queue_mutex);
	g_mutex_unlock(&priv->queue_mutex);

	for (i = 0; i < gst_buffer_n_memory(buffer->buffer); i++) {
		GstMemory *mem;
		mem = gst_buffer_peek_memory(buffer->buffer, i);
		if (!gst_is_dmabuf_memory(mem))
			continue;
		if (plane != expbuf->plane) {
			plane++;
			continue;
		}
		expbuf->fd = dup(gst_dmabuf_memory_get_fd (mem));
		return 0;
	}
	return -1;
}
