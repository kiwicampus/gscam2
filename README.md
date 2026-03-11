# gscam2 ![ROS2 CI](https://github.com/clydemcqueen/gscam2/actions/workflows/build_test.yml/badge.svg?branch=main)

ROS2 port of [gscam](https://github.com/ros-drivers/gscam).
Supports [ROS2 intra-process comms](https://docs.ros.org/en/humble/Tutorials/Demos/Intra-Process-Communication.html).

> Update 15-Jun-22: [gscam](https://index.ros.org/p/gscam/github-ros-drivers-gscam/) has been released for ROS2. It does not support IPC.

> Update 16-Mar-21: the package name has been updated to match the repository name (gscam2)

## Install and build

Tested on ROS2 Humble, Jazzy and Kilted.
See the [Dockerfile](Dockerfile) for install and build instructions.

## Usage

Make sure your GStreamer pipeline runs successfully in gst-launch-1.0.
For example, here's a pipeline that works for the [Blue Robotics HD USB Camera](https://bluerobotics.com/store/sensors-sonars-cameras/cameras/cam-usb-low-light-r1/):
~~~
gst-launch-1.0 -v v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! queue ! avdec_h264 ! autovideosink
~~~

Here's the same pipeline in gscam2:
~~~
export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert"
ros2 run gscam2 gscam_main
~~~

Here's an example with parameters:
~~~
ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/my_camera/image_raw --params-file gscam_params.yaml -p camera_info_url:=file://$PWD/my_camera_info.ini
~~~
... where gscam_params.yaml is:
~~~
gscam_publisher:
  ros__parameters:
    gscam_config: 'v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert'
    preroll: True
    use_gst_timestamps: True
    camera_name: 'my_camera'
    frame_id: 'my_camera_frame'
~~~

Here's an example that uses a GStreamer tee to split the stream, with one stream producing ROS images
and the second stream writing to MP4 files:
~~~
export GSCAM_CONFIG="v4l2src device=/dev/video1 do-timestamp=true ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! h264parse ! tee name=fork ! queue ! splitmuxsink location=video%02d.mov max-size-bytes=10000000 fork. ! avdec_h264 ! videoconvert"
ros2 run gscam2 gscam_main
~~~
There's a [bug](https://github.com/clydemcqueen/gscam2/issues/4) where the last MP4 file is not closed correctly.

### Intra-process comms

IPC test -- CLI composition:
~~~
# Create the container in the first shell; output will appear here as nodes are loaded:
export GSCAM_CONFIG="videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert"
ros2 run rclcpp_components component_container

# Add the nodes to the container in the second shell:
ros2 component load /ComponentManager gscam2 gscam2::ImageSubscriberNode -e use_intra_process_comms:=true
ros2 component load /ComponentManager gscam2 gscam2::GSCamNode -e use_intra_process_comms:=true
~~~

Launch file composition:
~~~
ros2 launch gscam2 composition_launch.py
~~~

Manual composition -- handy for debugging:
~~~
export GSCAM_CONFIG="videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert"
ros2 run gscam2 ipc_test_main
~~~

### Finding GStreamer plugins

GStreamer scans various paths for plugins and builds a plugin registry.
[The search process is described here](https://gstreamer.freedesktop.org/documentation/gstreamer/gstregistry.html?gi-language=c).

gscam uses a parameter `gst_plugin_path` instead than the commandline option `--gst-plugin-path`.
The paths in `gst_plugin_path` are searched last, not first.

If you have custom plugins you may need to override the plugin path. Here's an example showing how this works:
~~~
# Disable default locations
export GST_PLUGIN_PATH=""
export GST_PLUGIN_SYSTEM_PATH=""

# Provide the pipeline configuration
export GSCAM_CONFIG=videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert

# Run gscam_main, providing a custom plugin path
ros2 run gscam2 gscam_main  --ros-args -p gst_plugin_path:="/home/me/myplugins"
~~~

### NVIDIA nvvidconv and NvBufSurface

When using **nvvidconv** (or other NVIDIA GStreamer elements), buffers may be **NvBufSurface** (GPU/DMABUF). gscam2 currently reads frames with **`gst_memory_map()`**, which works when the buffer is in CPU-addressable memory.

- **If the pipeline already gives you CPU memory:** Many setups (e.g. nvvidconv → appsink) negotiate or copy to system memory, so `gst_memory_map()` works and no change is needed. Use `image_encoding: "rgba8"` or `"bgra8"` for the typical nvvidconv output (RGBA/BGRx).
- **If you get NvBufSurface (DMABUF) at the sink:** Then the mapped pointer may be invalid or the buffer layout may be different (planes, pitch). To read those buffers you need the **NvBufSurface API** (DeepStream/Jetson): get the DMABUF fd from the GstBuffer, call `NvBufSurfaceFromFd()`, then `NvBufSurfaceMap()` (or `NvBufSurface2Raw()`) to get CPU-accessible data and copy it into the ROS message. That path is not implemented in gscam2 yet; it would require an optional dependency on `libnvbufsurface` and a code path that uses the fd when present.

**Building with NvBufSurface support (Jetson/DeepStream):** If the buffers that reach the appsink are NvBufSurface (DMABUF), build with the optional NvBufSurface path:

```bash
colcon build --packages-select gscam2 --cmake-args -DGSCAM2_USE_NVBUF=ON
```

Requirements when `GSCAM2_USE_NVBUF=ON`: `gstreamer-allocators-1.0` (for fd-backed memory), and `nvbufsurface` (Jetson multimedia API or DeepStream SDK). The node will then try to get the DMABUF fd from the GStreamer buffer, call `NvBufSurfaceFromFd`, and copy frame data with `NvBufSurface2Raw` into the ROS Image message. If the buffer is not fd-backed or NvBufSurface fails, it falls back to the usual `gst_memory_map()` path.

**Practical suggestion:** Try your pipeline as-is (e.g. `... ! nvvidconv ! video/x-raw,format=RGBA ! appsink` with `image_encoding: "rgba8"`). If you see correct images, the plugin may be handing CPU-friendly memory. If you see crashes or corrupt frames, build with `GSCAM2_USE_NVBUF=ON` so the node can read NvBufSurface buffers.

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `gst_plugin_path` | string | | Similar to `--gst-plugin-path`, searchs path for plugins |
| `gscam_config` | string | | GStreamer pipeline configuration |
| `sync_sink` | bool | True | Enable GstBaseSink synchronization |
| `preroll` | bool | False | Transition to GST_STATE_PLAYING twice |
| `use_gst_timestamps` | bool | False | Use gst time instead of ROS time |
| `image_encoding` | string | `sensor_msgs::image_encodings::RGB8` |  ROS image encoding |
| `camera_info_url` | string | | URL to camera info file, e.g., `file:///path/to/file` |
| `camera_name` | string | | Replaces `${NAME}` in the URL  |
| `frame_id` | string | camera_frame | Camera frame ID |
| `skip` | int | 0 | Skip n frames for each frame sent; useful for reducing frame rates |

## Topics
- `camera_info`
- `image_raw`
- `image_raw/compressed` - only if image is encoded as a jpeg stream 

## Camera info file formats

Uses the [ROS standard camera calibration formats](http://wiki.ros.org/camera_calibration_parsers?distro=melodic).
Files must end in `.ini` or `.yaml`.


