#include "gscam2/gscam_node.hpp"

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include <chrono>

#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace gscam2
{

//=============================================================================
// Parameters
//=============================================================================

struct GSCamContext
{
  std::string gst_plugin_path_;   // Additional plugin path
  std::string gscam_config_;      // GStreamer configuration string
  bool sync_sink_{};              // Sync to the clock
  bool preroll_{};                // Pre-fill buffers
  bool use_gst_timestamps_{};     // Use gst time instead of ROS time
  std::string image_encoding_;    // Image encoding
  std::string camera_info_url_;   // Location of the camera info file
  std::string camera_name_;       // Camera name
  std::string frame_id_;          // Camera frame id
  int64_t skip_{};                // Skip n frames, then send 1
  double publish_rate_{0.0};      // Max publish rate in Hz, 0 = unlimited
  bool publish_foxglove_compressed_video_{true};  // H.264: also publish foxglove_msgs/CompressedVideo
  std::string foxglove_compressed_video_topic_{"foxglove_compressed_video"};
  /// When false with image_encoding=h264 and Foxglove enabled, only foxglove_msgs/CompressedVideo is published
  /// (no sensor_msgs/CompressedImage on image_raw/compressed). Always true for image_encoding=jpeg.
  bool publish_image_raw_compressed_{false};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
};

//=============================================================================
// GSCamNode::impl
//=============================================================================

class GSCamNode::impl
{
  // ROS node
  rclcpp::Node * node_;

  // Manage camera info
  camera_info_manager::CameraInfoManager camera_info_manager_;

  // Gstreamer structures
  GstElement * pipeline_;
  GstElement * sink_;

  // We need to poll GStreamer to get data
  // Move this to its own thread to avoid blocking or slowing down the rclcpp::spin() thread
  std::thread pipeline_thread_;

  // Used to stop the pipeline thread
  std::atomic<bool> stop_signal_;
  std::atomic<bool> shutdown_signal_;

  // Discover width and height from the incoming data
  int width_, height_;

  // Calibration between ros::Time and gst timestamps
  GstClockTime time_offset_;

  // Counter used to implement the 'skip' parameter
  int64_t skip_count_;

  // Timestamp of the last published frame, used to enforce publish_rate
  std::chrono::steady_clock::time_point last_publish_time_{};

  // Publish images...
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;

  // ... or compressed images
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;

  // Foxglove Studio: foxglove_msgs/CompressedVideo (H.264 only)
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr foxglove_compressed_video_pub_;

  // Publish camera info
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Create gstreamer pipeline, return true if successful
  bool create_pipeline();

  // Delete gstreamer pipeline
  void delete_pipeline();

  // Process one frame
  void process_frame();

public:
  // Parameters
  GSCamContext cxt_;

  // Constructor
  explicit impl(rclcpp::Node * node)
  : node_(node),
    camera_info_manager_(node),
    pipeline_(nullptr),
    sink_(nullptr),
    stop_signal_(false),
    shutdown_signal_(false),
    width_(0),
    height_(0),
    time_offset_(0),
    skip_count_(0)
  {
  }

  ~impl()
  {
    shutdown();
  }

  // Start or re-start pipeline
  void shutdown();
  void restart();
};

bool GSCamNode::impl::create_pipeline()
{
  if (!gst_is_initialized()) {
    // Only need to do this once
    gst_init(nullptr, nullptr);
    RCLCPP_INFO(node_->get_logger(), "Gstreamer initialized");

    if (!cxt_.gst_plugin_path_.empty()) {
      auto registry = gst_registry_get();
      if (!registry) {
        RCLCPP_ERROR(node_->get_logger(), "Could not get registry, ignoring gst_plugin_path");
      } else {
        if (gst_registry_scan_path(registry, cxt_.gst_plugin_path_.c_str())) {
          RCLCPP_INFO(
            node_->get_logger(), "Scanned '%s' and added plugins to the registry",
            cxt_.gst_plugin_path_.c_str());
        } else {
          RCLCPP_WARN(
            node_->get_logger(), "Scanned '%s' for plugins, but the registry did not change",
            cxt_.gst_plugin_path_.c_str());
        }
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Gstreamer version: %s", gst_version_string());

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(cxt_.gscam_config_.c_str(), &error);
  if (!pipeline_) {
    RCLCPP_FATAL(node_->get_logger(), "%s", error->message);
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", nullptr);
  if (!sink_) {
    RCLCPP_FATAL(
      node_->get_logger(),
      "gst_element_factory_make('appsink') failed. Is gstreamer1.0-plugins-base installed?");
    return false;
  }
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (cxt_.image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::YUV422_YUY2) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "YUY2",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::RGBA8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGBA",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::BGRA8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "BGRx",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::BAYER_RGGB8) {
    caps = gst_caps_new_simple(
      "video/x-bayer",
      "format", G_TYPE_STRING, "rggb",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::BAYER_BGGR8) {
    caps = gst_caps_new_simple(
      "video/x-bayer",
      "format", G_TYPE_STRING, "bggr",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::BAYER_GBRG8) {
    caps = gst_caps_new_simple(
      "video/x-bayer",
      "format", G_TYPE_STRING, "gbrg",
      nullptr);
  } else if (cxt_.image_encoding_ == sensor_msgs::image_encodings::BAYER_GRBG8) {
    caps = gst_caps_new_simple(
      "video/x-bayer",
      "format", G_TYPE_STRING, "grbg",
      nullptr);
  } else if (cxt_.image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", nullptr, nullptr);
  } else if (cxt_.image_encoding_ == "h264") {
    caps = gst_caps_new_simple("video/x-h264", nullptr, nullptr);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sink_),
    (cxt_.sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement * outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(node_->get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "Cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(nullptr);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, nullptr);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(node_->get_logger(), "Cannot link launchpipe -> sink");
      return false;
    }
  }

  // Calibration between rclcpp::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = node_->now().nanoseconds() - ct;
  RCLCPP_INFO(node_->get_logger(), "Time offset: %ld", time_offset_);

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to pause stream, check gscam_config");
    return false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Stream is paused");
  }

  cinfo_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  foxglove_compressed_video_pub_.reset();
  jpeg_pub_.reset();
  if (cxt_.image_encoding_ == "jpeg" || cxt_.image_encoding_ == "h264") {
    const bool foxglove_h264 =
      cxt_.image_encoding_ == "h264" && cxt_.publish_foxglove_compressed_video_;
    if (!cxt_.publish_image_raw_compressed_ && cxt_.image_encoding_ == "jpeg") {
      RCLCPP_FATAL(
        node_->get_logger(),
        "publish_image_raw_compressed=false is invalid for image_encoding=jpeg");
      return false;
    }
    if (!cxt_.publish_image_raw_compressed_ && cxt_.image_encoding_ == "h264" && !foxglove_h264) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "publish_image_raw_compressed=false requires publish_foxglove_compressed_video=true when "
        "image_encoding=h264");
      return false;
    }
    if (cxt_.publish_image_raw_compressed_ || cxt_.image_encoding_ == "jpeg") {
      jpeg_pub_ =
        node_->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1);
    }
    if (foxglove_h264) {
      foxglove_compressed_video_pub_ =
        node_->create_publisher<foxglove_msgs::msg::CompressedVideo>(
        cxt_.foxglove_compressed_video_topic_, rclcpp::QoS(1).best_effort());
      RCLCPP_INFO(
        node_->get_logger(),
        "Publishing foxglove_msgs/CompressedVideo on topic '%s'%s",
        cxt_.foxglove_compressed_video_topic_.c_str(),
        jpeg_pub_ ? " (and sensor_msgs/CompressedImage on image_raw/compressed)" :
        " only (no image_raw/compressed)");
    }
    if (!jpeg_pub_ && !foxglove_compressed_video_pub_) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "No video publisher created (check image_encoding / Foxglove / publish_image_raw_compressed)");
      return false;
    }
  } else {
    camera_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
  }

  // Pre-roll camera if needed
  if (cxt_.preroll_) {
    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to play in preroll");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Stream is playing in preroll");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, nullptr, nullptr, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(node_->get_logger(), "failed to pause in preroll");
      return false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Stream is paused in preroll");
    }
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(), "Could not start stream!");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Pipeline running");
  return true;
}

void GSCamNode::impl::delete_pipeline()
{
  // Stop a running stream, or cleanup a stream that failed to fully start
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    if (!shutdown_signal_ && rclcpp::ok()) {
      RCLCPP_INFO(node_->get_logger(), "Pipeline deleted");
    }
  }
}

unsigned int bytes_per_pixel(const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return 3;
  } else if (
    encoding == sensor_msgs::image_encodings::RGBA8 ||
    encoding == sensor_msgs::image_encodings::BGRA8)
  {
    return 4;
  } else if (
    encoding == sensor_msgs::image_encodings::MONO8 ||
    encoding == sensor_msgs::image_encodings::BAYER_RGGB8 ||
    encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
    encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||
    encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
  {
    return 1;
  } else {
    // sensor_msgs::image_encodings::YUV422_YUY2
    return 2;
  }
}

void GSCamNode::impl::process_frame()
{
  // Use a timeout to allow graceful shutdown even when the pipeline stalls.
  GstSample * sample = gst_app_sink_try_pull_sample(
    GST_APP_SINK(sink_), 100 * GST_MSECOND);
  if (!sample) {
    return;
  }

  // Implement frame skipping: drop 'skip' frames, then process 1
  if (cxt_.skip_ > 0) {
    if (skip_count_ < cxt_.skip_) {
      ++skip_count_;
      gst_sample_unref(sample);
      return;
    }
    skip_count_ = 0;
  }

  if (cxt_.publish_rate_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_publish_time_).count() < 1.0 / cxt_.publish_rate_) {
      gst_sample_unref(sample);
      return;
    }
    last_publish_time_ = now;
  }

  GstBuffer * buf = gst_sample_get_buffer(sample);
  if (!buf) {
    RCLCPP_INFO(node_->get_logger(), "Stream ended, pause for 1s");
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
    gst_sample_unref(sample);
    return;
  }

  // Map the full buffer: encoders often use multiple GstMemory chunks; mapping only index 0
  // truncates H.264/JPEG payloads.
  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to map GstBuffer");
    gst_sample_unref(sample);
    return;
  }
  const gsize buf_size = map.size;
  guint8 * buf_data = map.data;
  GstClockTime bt = gst_element_get_base_time(pipeline_);

  // Cache width/height — resolution doesn't change mid-stream
  if (width_ == 0 || height_ == 0) {
    GstPad * pad = gst_element_get_static_pad(sink_, "sink");
    GstCaps * caps = gst_pad_get_current_caps(pad);
    GstStructure * structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);
    gst_caps_unref(caps);
    gst_object_unref(pad);
  }

  // Update header information
  camera_info_manager::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
  auto cinfo = std::make_unique<sensor_msgs::msg::CameraInfo>(cur_cinfo);
  if (cxt_.use_gst_timestamps_) {
    cinfo->header.stamp = rclcpp::Time(static_cast<int64_t>(buf->pts + bt + time_offset_));
  } else {
    cinfo->header.stamp = node_->now();
  }
  cinfo->header.frame_id = cxt_.frame_id_;

  if (cxt_.image_encoding_ == "jpeg" || cxt_.image_encoding_ == "h264") {
    if (jpeg_pub_) {
      auto img = std::make_unique<sensor_msgs::msg::CompressedImage>();
      img->header = cinfo->header;
      img->format = cxt_.image_encoding_;
      img->data.resize(buf_size);
      std::copy(buf_data, buf_data + buf_size, img->data.begin());
      jpeg_pub_->publish(std::move(img));
    }
    if (cxt_.image_encoding_ == "h264" && foxglove_compressed_video_pub_) {
      foxglove_msgs::msg::CompressedVideo fv;
      fv.timestamp = cinfo->header.stamp;
      fv.frame_id = cinfo->header.frame_id;
      fv.data.assign(buf_data, buf_data + buf_size);
      fv.format = "h264";
      foxglove_compressed_video_pub_->publish(fv);
    }
    cinfo_pub_->publish(std::move(cinfo));
  } else {
    const unsigned int expected_frame_size =
      width_ * height_ * bytes_per_pixel(cxt_.image_encoding_);

    if (buf_size < expected_frame_size) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Image buffer underflow: expected frame to be %d bytes but got only %lu"
        " bytes (make sure frames are correctly encoded)", expected_frame_size, buf_size);
    }

    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header = cinfo->header;
    img->width = width_;
    img->height = height_;
    img->encoding = cxt_.image_encoding_;
    img->is_bigendian = false;
    img->data.resize(expected_frame_size);
    img->step = width_ * bytes_per_pixel(cxt_.image_encoding_);
    std::copy(buf_data, buf_data + buf_size, img->data.begin());

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
    static int count = 0;
    RCLCPP_INFO(
      node_->get_logger(), "%d, %p", count++,
      reinterpret_cast<std::uintptr_t>(img.get()));
#endif

    camera_pub_->publish(std::move(img));
    cinfo_pub_->publish(std::move(cinfo));
  }

  gst_buffer_unmap(buf, &map);
  gst_sample_unref(sample);
}

void GSCamNode::impl::shutdown()
{
  if (shutdown_signal_.exchange(true)) {
    return;
  }

  if (pipeline_) {
    stop_signal_ = true;
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (pipeline_thread_.joinable()) {
      pipeline_thread_.join();
    }
    delete_pipeline();
  }
}

void GSCamNode::impl::restart()
{
  if (pipeline_) {
    // Stop thread
    stop_signal_ = true;
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (pipeline_thread_.joinable()) {
      pipeline_thread_.join();
    }

    // Delete pipeline
    delete_pipeline();
  }

  // If gscam_config is empty look for GSCAM_CONFIG in the environment
  if (cxt_.gscam_config_.empty()) {
    auto gsconfig_env = getenv("GSCAM_CONFIG");
    if (gsconfig_env) {
      RCLCPP_INFO(node_->get_logger(), "Using GSCAM_CONFIG env var: %s", gsconfig_env);
      cxt_.gscam_config_ = gsconfig_env;
    } else {
      RCLCPP_FATAL(
        node_->get_logger(),
        "GSCAM_CONFIG env var and gscam_config param are both missing, can't start stream");
      return;
    }
  }

  if (cxt_.image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::RGBA8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::BGRA8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::YUV422_YUY2 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::BAYER_RGGB8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::BAYER_BGGR8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::BAYER_GBRG8 &&
    cxt_.image_encoding_ != sensor_msgs::image_encodings::BAYER_GRBG8 &&
    cxt_.image_encoding_ != "jpeg" &&
    cxt_.image_encoding_ != "h264")
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Unsupported image encoding: %s",
      cxt_.image_encoding_.c_str());
    return;
  }

  camera_info_manager_.setCameraName(cxt_.camera_name_);

  if (camera_info_manager_.validateURL(cxt_.camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(cxt_.camera_info_url_);
    RCLCPP_INFO(
      node_->get_logger(), "Loaded camera calibration from %s", cxt_.camera_info_url_.c_str());
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Camera info url '%s' is not valid, missing 'file://' prefix?",
      cxt_.camera_info_url_.c_str());
  }

  // Reset cached state before (re)starting the pipeline
  width_ = 0;
  height_ = 0;
  last_publish_time_ = {};

  // [Re-]start the pipeline in its own thread
  if (create_pipeline()) {
    pipeline_thread_ = std::thread(
      [this]()
      {
        if (!shutdown_signal_ && rclcpp::ok()) {
          RCLCPP_INFO(node_->get_logger(), "Thread running");    // NOLINT
        }

        // reset skipping state when (re)starting
        skip_count_ = 0;

        while (!stop_signal_ && rclcpp::ok()) {
          process_frame();
        }

        stop_signal_ = false;
        if (!shutdown_signal_ && rclcpp::ok()) {
          RCLCPP_INFO(node_->get_logger(), "Thread stopped");    // NOLINT
        }
      });
  } else {
    delete_pipeline();
  }
}

//=============================================================================
// GSCamNode
//=============================================================================

GSCamNode::GSCamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  pImpl_(std::make_unique<GSCamNode::impl>(this))
{
  RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());
  on_shutdown_handle_ = get_node_base_interface()->get_context()->add_on_shutdown_callback(
    [this]() {pImpl_->shutdown();});

  // Declare and get parameters
  pImpl_->cxt_.gst_plugin_path_ = declare_parameter("gst_plugin_path", "");
  pImpl_->cxt_.gscam_config_ = declare_parameter("gscam_config", "");
  pImpl_->cxt_.sync_sink_ = declare_parameter("sync_sink", true);
  pImpl_->cxt_.preroll_ = declare_parameter("preroll", false);
  pImpl_->cxt_.use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);
  pImpl_->cxt_.image_encoding_ = declare_parameter(
    "image_encoding",
    sensor_msgs::image_encodings::RGB8);
  pImpl_->cxt_.camera_info_url_ = declare_parameter("camera_info_url", "");
  pImpl_->cxt_.camera_name_ = declare_parameter("camera_name", "");
  pImpl_->cxt_.frame_id_ = declare_parameter("frame_id", "camera_frame");
  pImpl_->cxt_.skip_ = declare_parameter("skip", 0);
  pImpl_->cxt_.publish_rate_ = declare_parameter("publish_rate", 0.0);
  pImpl_->cxt_.publish_foxglove_compressed_video_ =
    declare_parameter("publish_foxglove_compressed_video", true);
  pImpl_->cxt_.foxglove_compressed_video_topic_ =
    declare_parameter("foxglove_compressed_video_topic", std::string("foxglove_compressed_video"));
  pImpl_->cxt_.publish_image_raw_compressed_ =
    declare_parameter("publish_image_raw_compressed", false);

  validate_parameters();

  // Register parameters
  pImpl_->cxt_.on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::
    SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      bool param_set = false;

      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "gst_plugin_path") {
          pImpl_->cxt_.gst_plugin_path_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "gscam_config") {
          pImpl_->cxt_.gscam_config_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "sync_sink") {
          pImpl_->cxt_.sync_sink_ = parameter.as_bool();
          param_set = true;
        } else if (parameter.get_name() == "preroll") {
          pImpl_->cxt_.preroll_ = parameter.as_bool();
          param_set = true;
        } else if (parameter.get_name() == "use_gst_timestamps") {
          pImpl_->cxt_.use_gst_timestamps_ = parameter.as_bool();
          param_set = true;
        } else if (parameter.get_name() == "image_encoding") {
          pImpl_->cxt_.image_encoding_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "camera_info_url") {
          pImpl_->cxt_.camera_info_url_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "camera_name") {
          pImpl_->cxt_.camera_name_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "frame_id") {
          pImpl_->cxt_.frame_id_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "skip") {
          pImpl_->cxt_.skip_ = parameter.as_int();
          param_set = true;
        } else if (parameter.get_name() == "publish_rate") {
          pImpl_->cxt_.publish_rate_ = parameter.as_double();
          // No pipeline restart needed — throttle takes effect immediately
        } else if (parameter.get_name() == "publish_foxglove_compressed_video") {
          pImpl_->cxt_.publish_foxglove_compressed_video_ = parameter.as_bool();
          param_set = true;
        } else if (parameter.get_name() == "foxglove_compressed_video_topic") {
          pImpl_->cxt_.foxglove_compressed_video_topic_ = parameter.as_string();
          param_set = true;
        } else if (parameter.get_name() == "publish_image_raw_compressed") {
          pImpl_->cxt_.publish_image_raw_compressed_ = parameter.as_bool();
          param_set = true;
        }

        if (param_set) {
          RCLCPP_INFO(get_logger(), "Parameter %s value changed", parameter.get_name().c_str());
        }
      }

      if (param_set) {
        validate_parameters();
      }
      return result;
    }
  );
}

GSCamNode::~GSCamNode()
{
  if (auto context = get_node_base_interface()->get_context()) {
    context->remove_on_shutdown_callback(on_shutdown_handle_);
  }
  pImpl_.reset();
}

void GSCamNode::validate_parameters()
{
  RCLCPP_INFO(get_logger(), "gst_plugin_path = %s", pImpl_->cxt_.gst_plugin_path_.c_str());
  RCLCPP_INFO(get_logger(), "gscam_config = %s", pImpl_->cxt_.gscam_config_.c_str());
  RCLCPP_INFO(get_logger(), "sync_sink = %s", pImpl_->cxt_.sync_sink_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "preroll = %s", pImpl_->cxt_.preroll_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "use_gst_timestamps = %s",
    pImpl_->cxt_.use_gst_timestamps_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "image_encoding = %s", pImpl_->cxt_.image_encoding_.c_str());
  RCLCPP_INFO(get_logger(), "camera_info_url = %s", pImpl_->cxt_.camera_info_url_.c_str());
  RCLCPP_INFO(get_logger(), "camera_name = %s", pImpl_->cxt_.camera_name_.c_str());
  RCLCPP_INFO(get_logger(), "frame_id = %s", pImpl_->cxt_.frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "skip = %ld", pImpl_->cxt_.skip_);
  RCLCPP_INFO(get_logger(), "publish_rate = %.1f Hz (0 = unlimited)", pImpl_->cxt_.publish_rate_);
  RCLCPP_INFO(
    get_logger(), "publish_foxglove_compressed_video = %s",
    pImpl_->cxt_.publish_foxglove_compressed_video_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "foxglove_compressed_video_topic = %s",
    pImpl_->cxt_.foxglove_compressed_video_topic_.c_str());
  RCLCPP_INFO(
    get_logger(), "publish_image_raw_compressed = %s",
    pImpl_->cxt_.publish_image_raw_compressed_ ? "true" : "false");

  pImpl_->restart();
}

} // namespace gscam2

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam2::GSCamNode)  // NOLINT
