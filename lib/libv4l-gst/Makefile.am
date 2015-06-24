libv4l2plugin_LTLIBRARIES = libv4l-gst.la

include_HEADERS = $(top_srcdir)/lib/include/libv4l-gst-bufferpool.h

libv4l_gst_la_SOURCES = libv4l-gst.c
libv4l_gst_la_CFLAGS = $(GST_CFLAGS)
libv4l_gst_la_CPPFLAGS = $(CFLAG_VISIBILITY)
libv4l_gst_la_LIBADD = $(GST_LIBS)
libv4l_gst_la_LDFLAGS = -avoid-version -module -shared -export-dynamic -lpthread
