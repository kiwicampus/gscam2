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

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `gst_plugin_path` | string | | Similar to `--gst-plugin-path`, searchs path for plugins |
| `gscam_config` | string | | GStreamer pipeline configuration |
| `sync_sink` | bool | True | Enable GstBaseSink synchronization |
| `preroll` | bool | False | Transition to GST_STATE_PLAYING twice |
| `use_gst_timestamps` | bool | False | Use gst time instead of ROS time |
| `image_encoding` | string | `sensor_msgs::image_encodings::RGB8` |  ROS image encoding; use `"jpeg"` or `"h264"` for compressed streams |
| `camera_info_url` | string | | URL to camera info file, e.g., `file:///path/to/file` |
| `camera_name` | string | | Replaces `${NAME}` in the URL  |
| `frame_id` | string | camera_frame | Camera frame ID |
| `skip` | int | 0 | Skip n frames for each frame sent; useful for reducing frame rates |
| `publish_foxglove_compressed_video` | bool | true | If `image_encoding` is `h264`, also publish `foxglove_msgs/CompressedVideo` for Foxglove Studio |
| `foxglove_compressed_video_topic` | string | `foxglove_compressed_video` | Topic name for `foxglove_msgs/CompressedVideo` |

## Topics
- `camera_info`
- `image_raw` — uncompressed (`sensor_msgs/Image`)
- `image_raw/compressed` — if `image_encoding` is `jpeg` or `h264` (`sensor_msgs/CompressedImage` with `format` set to `jpeg` or `h264`)
- **`foxglove_compressed_video`** (default name) — if `image_encoding` is `h264` and `publish_foxglove_compressed_video` is true: **`foxglove_msgs/CompressedVideo`** with `format: "h264"` for [Foxglove Studio](https://foxglove.dev/) Image / 3D panels

### H.264 and “compressed video”

In ROS 2 there is **no separate standard message** named “CompressedVideo”. **H.264 is published on `image_raw/compressed` as `sensor_msgs/CompressedImage`**: the `data` field carries the encoded bitstream (NAL/access units) and `format` is the string `"h264"`. That is the usual pattern for any compressed payload (JPEG, PNG, H.264, etc.).

So **yes, H.264 goes with “compressed”** in the sense of `CompressedImage` — not with `sensor_msgs/Image` (which is for decoded pixels).

What often breaks expectations:

- **`image_transport`** republishers and many tools only treat `format: jpeg` or `png` by default. They may **ignore or fail on `format: h264`** unless you add a plugin or a custom subscriber that decodes H.264.
- The topic name `.../compressed` is the same as for JPEG; the **codec is only in `msg.format`**, not in the topic type.

Set `image_encoding: "h264"` and use a pipeline that outputs `video/x-h264` to the appsink, for example:

```yaml
gscam_config: "v4l2src device=/dev/video0 ! video/x-h264 ! appsink"
# or: rtspsrc location=... ! rtph264depay ! video/x-h264 ! appsink
image_encoding: "h264"
```

Subscribers must explicitly handle `CompressedImage` with `format == "h264"` (e.g. GStreamer, FFmpeg, hardware decoder, or a node that republishes decoded `sensor_msgs/Image`).

### Foxglove Studio (CompressedVideo)

Foxglove expects **`foxglove_msgs/msg/CompressedVideo`**, not `sensor_msgs/CompressedImage`. With `image_encoding: "h264"`, gscam2 publishes **both**: ROS `CompressedImage` on `image_raw/compressed` and **`foxglove_msgs/CompressedVideo`** on `foxglove_compressed_video` (override with `foxglove_compressed_video_topic`). In Foxglove, add an **Image** panel and subscribe to that topic; the schema is detected automatically over a ROS 2 connection.

Foxglove’s docs recommend **Annex B** H.264 (`0x00 0x00 0x01` start codes). If playback fails, try forcing byte-stream in the pipeline, e.g. `h264parse ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink`. Each message should represent **one decodable frame**; keyframes should include **SPS** (and typically **PPS**) NAL units. Foxglove does not support H.264 with **B-frames** (no lookahead).

## Camera info file formats

Uses the [ROS standard camera calibration formats](http://wiki.ros.org/camera_calibration_parsers?distro=melodic).
Files must end in `.ini` or `.yaml`.


