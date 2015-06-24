#ifndef __LIBV4L_GST_BUFFER_POOL_H__
#define __LIBV4L_GST_BUFFER_POOL_H__

#include <gst/gst.h>

struct libv4l_gst_buffer_pool_ops {
	GstBufferPool* (*add_external_src_buffer_pool)(void);
	GstBufferPool* (*add_external_sink_buffer_pool)(void);
};

#endif /*__LIBV4L_GST_BUFFER_POOL__*/
