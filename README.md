Overview
========

This package adds a V4L2 plugin that creates a bridge between the V4L2 API and
GStreamer for mem-to-mem decoder components. This plugin has not been tested
in device-to-mem or mem-to-device configurations. Applications that use the
libv4l API to perform codec operations should be able to use this plugin to
connect to a GStreamer pipeline instead of a V4L hardware device without any
updates.

Dependencies
============

* v4lutils - with patches provide [here](https://github.com/igel-oss/v4l-utils)
* [v4l-gst-bufferpool-rcar] (https://github.com/igel-oss/v4l-gst-bufferpool-rcar) for use with Renesas R-Car boards (i.e Porter)

Compile
=======

```
$ autoreconf -vif
$ ./configure
```

Configuration
=============
The setting file location is `/etc/xdg/libv4l-gst.conf`

### Settings

**pipeline**: The GStreamer pipeline to be used.  Should include everything but the input and output nodes  
**max-width**: The maximum width of the video that can be decoded through the plugin (default: 1920)  
**max-height**: The maximum width of the video that can be decoded through the plugin (default: 1080)   
**bufferpool-library**: Path to the library that provides buffer pools for input and output nodes  
**min-buffers**: The minimum number of buffers for each of the above buffer pools  

### Example
The following settings are for the Renesas Porter board,
but they may be updated to use more generic settings.


```
[libv4l-gst]
pipeline=h264parse ! omxh264dec ! queue max-size-bytes=0 max-size-time=0 max-size-buffers=0 ! vspfilter
bufferpool-library=/usr/lib/libv4l/plugins/v4l-gst-bufferpool/libv4l-gst-bufferpool-rel.so
min-buffers=2
```

Running
=======

Create a dummy V4L2 device file under /dev
```
# touch /dev/video-gst
# chmod 666 /dev/video-gst
```
Accessing the /dev/video-gst file will allow an application to use the v4l-gst plugin
using the same API as a regular V4L2 device file.

