# -*- Autoconf -*-

# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT([libv4l-gst],[1.0.0])

AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile
	lib/Makefile
	lib/libv4l-gst/Makefile
])

AC_GNU_SOURCE

AM_INIT_AUTOMAKE([1.9 subdir-objects no-dist-gzip dist-bzip2 -Wno-portability])

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

# Checks for programs.
AC_PROG_CC_C99
AC_PROG_INSTALL
AC_PROG_LIBTOOL
AC_PROG_MKDIR_P

AC_CHECK_HEADER([libv4l-plugin.h], [], AC_MSG_ERROR([libv4l-plugin.h is needed]))
PKG_PROG_PKG_CONFIG
PKG_CHECK_MODULES(GST, gstreamer-video-1.0 >= 1.4 gstreamer-app-1.0
		gstreamer-allocators-1.0,
	[AC_DEFINE([GST_14], [1], [Gstreamer version 1.4])],
	[PKG_CHECK_MODULES(GST, gstreamer-video-1.0 gstreamer-app-1.0
		gstreamer-allocators-1.0)])

AC_ARG_WITH(libv4l2subdir,
        AS_HELP_STRING([--with-libv4l2subdir=DIR], [set libv4l2 library subdir (default=libv4l)]),
        [],
        [with_libv4l2subdir="libv4l"])

AC_SUBST([libv4l2plugindir], [$libdir/$with_libv4l2subdir/plugins])

AC_DEFINE_DIR([LIBV4L2_PLUGIN_DIR], [libv4l2plugindir], [libv4l2 plugin directory])

AC_ARG_ENABLE([chromium-compatibility],
	[AS_HELP_STRING([--enable-chromium-compatibility],
			[Enable the specific operation to link with the Chromium V4L2 VDA])],
	[],
	[enable_chromium_compatibility=no]
)

if test "x${enable_chromium_compatibility}" = "xyes"; then
	AC_DEFINE(ENABLE_CHROMIUM_COMPAT, 1,
		[Define if the specific operation to link with the Chromium V4L2 VDA is enabled])
fi

CPPFLAGS="-I\$(top_srcdir)/lib/include -Wall -Wpointer-arith $CPPFLAGS"

AC_OUTPUT
