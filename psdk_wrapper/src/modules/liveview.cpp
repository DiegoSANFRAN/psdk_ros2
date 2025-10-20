/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file liveview.cpp
 *
 * @brief Liveview module implementation. This module is responsible for
 * handling the liveview stream from the drone's cameras.
 *
 * @authors Lidia de la Torre Vazquez, Bianca Bendris
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/modules/liveview.hpp"

// GStreamer headers (C linkage)
extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
}
#include <vector>
#include <cstring>

namespace psdk_ros2
{
LiveviewModule::LiveviewModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating LiveviewModule");
}

LiveviewModule::~LiveviewModule()
{
  RCLCPP_INFO(get_logger(), "Destroying LiveviewModule");
}

LiveviewModule::CallbackReturn
LiveviewModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring LiveviewModule");
  
  // Declare and get keyframe request interval parameter
  // Default 0.0 means disabled, 0.25 = 4Hz I-frames
  this->declare_parameter("auto_keyframe_interval", 2.0);
  keyframe_request_interval_ = this->get_parameter("auto_keyframe_interval").as_double();
  auto_keyframe_enabled_ = (keyframe_request_interval_ > 0.0);

  // Direct RTP parameters
  this->declare_parameter("direct_rtp.enabled", true);
  this->declare_parameter("direct_rtp.host", std::string("127.0.0.1"));
  this->declare_parameter("direct_rtp.port", 5006);
  this->declare_parameter("direct_rtp.pt", 96);
  this->declare_parameter("direct_rtp.ssrc", 11111111);
  this->declare_parameter("direct_rtp.mtu", 1400);  // Increased from 1000 for efficiency
  this->declare_parameter("direct_rtp.iframes_only", false);
  direct_rtp_enabled_ = this->get_parameter("direct_rtp.enabled").as_bool();
  rtp_host_ = this->get_parameter("direct_rtp.host").as_string();
  rtp_port_ = this->get_parameter("direct_rtp.port").as_int();
  rtp_pt_ = this->get_parameter("direct_rtp.pt").as_int();
  rtp_ssrc_ = this->get_parameter("direct_rtp.ssrc").as_int();
  rtp_mtu_ = this->get_parameter("direct_rtp.mtu").as_int();
  direct_iframes_only_ = this->get_parameter("direct_rtp.iframes_only").as_bool();
  
  if (auto_keyframe_enabled_)
  {
    RCLCPP_INFO(get_logger(), 
                "🎯 Automatic keyframe requests ENABLED: %.2f Hz (every %.3f seconds)",
                1.0 / keyframe_request_interval_, keyframe_request_interval_);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Automatic keyframe requests DISABLED (interval=0)");
  }

  if (direct_rtp_enabled_)
  {
    RCLCPP_INFO(get_logger(),
                "✅ Direct RTP mode ENABLED → %s:%d, PT=%u, SSRC=%u, MTU=%d, I-frames only=%s",
                rtp_host_.c_str(), rtp_port_, rtp_pt_, rtp_ssrc_, rtp_mtu_,
                direct_iframes_only_ ? "true" : "false");
    // Initialize GStreamer once (safe to call multiple times)
    static bool gst_inited = false;
    if (!gst_inited)
    {
      int argc = 0; char** argv = nullptr;
      gst_init(&argc, &argv);
      gst_inited = true;
    }
  }
  
  main_camera_stream_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "psdk_ros2/main_camera_stream", rclcpp::SensorDataQoS());
  fpv_camera_stream_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "psdk_ros2/fpv_camera_stream", rclcpp::SensorDataQoS());
  camera_setup_streaming_service_ = create_service<CameraSetupStreaming>(
      "psdk_ros2/camera_setup_streaming",
      std::bind(&LiveviewModule::camera_setup_streaming_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_request_intraframe_service_ = create_service<CameraRequestIntraframe>(
      "psdk_ros2/camera_request_intraframe",
      std::bind(&LiveviewModule::camera_request_intraframe_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating LiveviewModule");
  main_camera_stream_pub_->on_activate();
  fpv_camera_stream_pub_->on_activate();
  
  // Note: Automatic keyframe timer will be started when streaming actually begins
  // (see camera_setup_streaming_cb with start_stop=true)
  if (auto_keyframe_enabled_)
  {
    RCLCPP_INFO(get_logger(), 
                "Automatic keyframe requests configured at %.2f Hz - will start when streaming begins",
                1.0 / keyframe_request_interval_);
  }
  
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating LiveviewModule");
  
  // Stop automatic keyframe request timer
  if (keyframe_request_timer_)
  {
    keyframe_request_timer_->cancel();
    keyframe_request_timer_.reset();
    RCLCPP_INFO(get_logger(), "Stopped automatic keyframe request timer");
  }
  
  main_camera_stream_pub_->on_deactivate();
  fpv_camera_stream_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up LiveviewModule");
  // Ensure pipeline is stopped
  stop_rtp_pipeline();
  camera_setup_streaming_service_.reset();
  camera_request_intraframe_service_.reset();
  main_camera_stream_pub_.reset();
  fpv_camera_stream_pub_.reset();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down LiveviewModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_liveview_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
LiveviewModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Liveview module is already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating liveview module");
  T_DjiReturnCode return_code = DjiLiveview_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  /* Start decoders*/
  stream_decoder_ = {
      {DJI_LIVEVIEW_CAMERA_POSITION_FPV, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
  };
  decode_stream_ = true;
  payload_index_ = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
  is_module_initialized_ = true;
  return true;
}

bool
LiveviewModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing liveview module");
  T_DjiReturnCode return_code = DjiLiveview_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buffer,
                                   uint32_t buffer_length)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);

  if (global_liveview_ptr_->decode_stream_)
  {
    return global_liveview_ptr_->LiveviewConvertH264ToRgbCallback(
        position, buffer, buffer_length);
  }
  // Direct RTP pipeline: push H264 frames directly to appsrc
  if (global_liveview_ptr_->direct_rtp_enabled_ && global_liveview_ptr_->gst_pipeline_)
  {
    global_liveview_ptr_->push_h264_to_pipeline(buffer, buffer_length);
    return;
  }
  // Fallback: publish on ROS2 topic
  if (global_liveview_ptr_->payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
  {
    return global_liveview_ptr_->publish_fpv_camera_images(buffer, buffer_length);
  }
  return global_liveview_ptr_->publish_main_camera_images(buffer, buffer_length);
}

void
LiveviewModule::LiveviewConvertH264ToRgbCallback(
    E_DjiLiveViewCameraPosition position, const uint8_t *buffer,
    uint32_t buffer_length)
{
  auto decoder = stream_decoder_.find(position);
  if ((decoder != stream_decoder_.end()) && decoder->second)
  {
    decoder->second->decodeBuffer(buffer, buffer_length);
  }
}

void
c_publish_main_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_main_camera_images(img, user_data);
}

void
c_publish_fpv_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_fpv_camera_images(img, user_data);
}

void
LiveviewModule::camera_setup_streaming_cb(
    const std::shared_ptr<CameraSetupStreaming::Request> request,
    const std::shared_ptr<CameraSetupStreaming::Response> response)
{
  selected_camera_source_ =
      static_cast<E_DjiLiveViewCameraSource>(request->camera_source);
  decode_stream_ = request->decoded_output;
  payload_index_ =
      static_cast<E_DjiLiveViewCameraPosition>(request->payload_index);

  RCLCPP_INFO(get_logger(),
              "Setting up camera streaming for payload index %d and camera "
              "source %d. Output decoded: %d",
              payload_index_, selected_camera_source_, decode_stream_);

  if (request->start_stop)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming...");
    bool streaming_result;
    if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_NO_1)
    {
      char main_camera_name[] = "MAIN_CAMERA";
      streaming_result = start_camera_stream(&c_publish_main_streaming_callback,
                                             &main_camera_name, payload_index_,
                                             selected_camera_source_);
    }
    else if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
    {
      char fpv_camera_name[] = "FPV_CAMERA";
      streaming_result = start_camera_stream(&c_publish_fpv_streaming_callback,
                                             &fpv_camera_name, payload_index_,
                                             selected_camera_source_);
    }

    if (streaming_result)
    {
      is_streaming_active_ = true;

      // Start RTP pipeline if enabled
      if (direct_rtp_enabled_)
      {
        if (!start_rtp_pipeline())
        {
          RCLCPP_ERROR(get_logger(), "Failed to start direct RTP pipeline; falling back to ROS2 topics");
        }
      }
      
      // Start automatic keyframe timer now that streaming is active
      if (auto_keyframe_enabled_ && !keyframe_request_timer_)
      {
        keyframe_request_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(keyframe_request_interval_),
            std::bind(&LiveviewModule::auto_request_keyframe_callback, this));
        RCLCPP_INFO(get_logger(), 
                    "🎯 Started automatic keyframe request timer: %.2f Hz (every %.3f seconds)",
                    1.0 / keyframe_request_interval_, keyframe_request_interval_);
      }
      
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Stopping camera streaming...");
    // Stop RTP pipeline if running
    stop_rtp_pipeline();
    
    // Stop automatic keyframe timer when streaming stops
    if (keyframe_request_timer_)
    {
      keyframe_request_timer_->cancel();
      keyframe_request_timer_.reset();
      RCLCPP_INFO(get_logger(), "Stopped automatic keyframe request timer");
    }
    is_streaming_active_ = false;
    
    if (stop_main_camera_stream(payload_index_, selected_camera_source_))
    {
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
}

void
LiveviewModule::camera_request_intraframe_cb(
    const std::shared_ptr<CameraRequestIntraframe::Request> request,
    const std::shared_ptr<CameraRequestIntraframe::Response> response)
{
  E_DjiLiveViewCameraPosition payload_index =
      static_cast<E_DjiLiveViewCameraPosition>(request->payload_index);
  E_DjiLiveViewCameraSource camera_source =
      static_cast<E_DjiLiveViewCameraSource>(request->camera_source);

  RCLCPP_INFO(get_logger(),
              "Requesting intraframe for payload_index=%d, camera_source=%d",
              payload_index, camera_source);

  T_DjiReturnCode return_code = DjiLiveview_RequestIntraframeFrameData(
      payload_index, camera_source);
  
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_WARN(get_logger(),
                "Intraframe request failed, error code: %ld", return_code);
    response->success = false;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Intraframe request succeeded");
    response->success = true;
  }
}

bool
LiveviewModule::start_camera_stream(CameraImageCallback callback,
                                    void *user_data,
                                    E_DjiLiveViewCameraPosition payload_index,
                                    E_DjiLiveViewCameraSource camera_source)
{
  if (decode_stream_)
  {
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->init();
      decoder->second->registerCallback(callback, user_data);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to set-up the decoder");
      return false;
    }
  }

  T_DjiReturnCode return_code = DjiLiveview_StartH264Stream(
      payload_index, camera_source, c_LiveviewConvertH264ToRgbCallback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to start camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully started the camera streaming.");

    // Request intraframe multiple times to ensure IDR delivery (especially for FPV)
    for (int attempt = 0; attempt < 3; ++attempt)
    {
      T_DjiReturnCode intraframe_code = DjiLiveview_RequestIntraframeFrameData(
          payload_index, camera_source);
      if (intraframe_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
      {
        RCLCPP_INFO(get_logger(),
                    "Requested intraframe (attempt %d) for payload index %d, camera source %d.",
                    attempt + 1, payload_index, camera_source);
      }
      else
      {
        RCLCPP_WARN(get_logger(),
                    "Intraframe request attempt %d failed, error code: %ld.",
                    attempt + 1, intraframe_code);
      }
      // Brief delay between requests
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
  }
}

bool
LiveviewModule::stop_main_camera_stream(
    const E_DjiLiveViewCameraPosition payload_index,
    const E_DjiLiveViewCameraSource camera_source)
{
  T_DjiReturnCode return_code =
      DjiLiveview_StopH264Stream(payload_index, camera_source);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to stop camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->cleanup();
    }
    RCLCPP_INFO(get_logger(), "Successfully stopped camera streaming.");
    return true;
  }
}

void
LiveviewModule::publish_main_camera_images(const uint8_t *buffer,
                                           uint32_t buffer_length)
{
  if (direct_rtp_enabled_) return;  // suppressed in direct mode
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = get_optical_frame_id();
  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_fpv_camera_images(const uint8_t *buffer,
                                          uint32_t buffer_length)
{
  if (direct_rtp_enabled_) return;  // suppressed in direct mode
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  fpv_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_main_camera_images(CameraRGBImage rgb_img,
                                           void *user_data)
{
  (void)user_data;
  // Suppress RGB topic publishing when direct RTP mode is enabled
  if (direct_rtp_enabled_) return;
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->height = rgb_img.height;
  img->width = rgb_img.width;
  img->step = rgb_img.width * 3;
  img->encoding = "rgb8";
  img->data = rgb_img.rawData;

  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = get_optical_frame_id();
  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_fpv_camera_images(CameraRGBImage rgb_img,
                                          void *user_data)
{
  (void)user_data;
  // Suppress RGB topic publishing when direct RTP mode is enabled
  if (direct_rtp_enabled_) return;
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->height = rgb_img.height;
  img->width = rgb_img.width;
  img->step = rgb_img.width * 3;
  img->encoding = "rgb8";
  img->data = rgb_img.rawData;

  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  fpv_camera_stream_pub_->publish(std::move(img));
}

std::string
LiveviewModule::get_optical_frame_id()
{
  for (auto &it : psdk_utils::camera_source_str)
  {
    if (it.first == selected_camera_source_)
    {
      return it.second;
    }
  }
}

// ===================== GStreamer Direct RTP Impl =====================
bool LiveviewModule::start_rtp_pipeline()
{
  if (!direct_rtp_enabled_) return false;
  if (gst_pipeline_) return true; // already running

  // Build pipeline in code: appsrc ! h264parse ! rtph264pay ! udpsink
  GstElement* pipeline = gst_pipeline_new("psdk_rtp_pipeline");
  if (!pipeline)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline");
    return false;
  }
  GstElement* appsrc = gst_element_factory_make("appsrc", "src");
  GstElement* parse = gst_element_factory_make("h264parse", "parse");
  GstElement* queue = gst_element_factory_make("queue", "queue");
  GstElement* pay = gst_element_factory_make("rtph264pay", "pay");
  GstElement* sink = gst_element_factory_make("udpsink", "sink");
  if (!appsrc || !parse || !queue || !pay || !sink)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create one or more GStreamer elements");
    if (pipeline) gst_object_unref(pipeline);
    return false;
  }
  
  // Configure queue: small buffer, leaky=downstream (drop old frames), no blocking
  g_object_set(G_OBJECT(queue),
               "max-size-buffers", 5,  // Only keep 5 frames
               "max-size-bytes", 0,
               "max-size-time", 0,
               "leaky", 2,  // GST_QUEUE_LEAK_DOWNSTREAM (drop oldest)
               "flush-on-eos", TRUE,
               nullptr);

  // Configure elements
  // appsrc: live, time, block=false, emit-signals=false, max-bytes=0 (unlimited but fast drops)
  g_object_set(G_OBJECT(appsrc),
               "is-live", TRUE,
               "format", GST_FORMAT_TIME,
               "block", FALSE,
               "emit-signals", FALSE,
               "max-bytes", 0,
               nullptr);
  // h264parse: prefer AU alignment and allow passthrough
  g_object_set(G_OBJECT(parse),
               "disable-passthrough", FALSE,
               "alignment", 1 /* au */, // GST_H264_PARSE_ALIGNMENT_AU
               nullptr);
  // rtph264pay: pt, ssrc, mtu, config-interval=-1, aggregate-mode=zero-latency
  g_object_set(G_OBJECT(pay),
               "pt", rtp_pt_,
               "ssrc", rtp_ssrc_,
               "mtu", rtp_mtu_,
               "config-interval", -1,
               "aggregate-mode", 1, // zero-latency (don't wait to aggregate)
               nullptr);
  // udpsink: host/port, sync/async false, max-bitrate=0 (unlimited), max-lateness=-1 (drop late)
  g_object_set(G_OBJECT(sink),
               "host", rtp_host_.c_str(),
               "port", rtp_port_,
               "sync", FALSE,
               "async", FALSE,
               "max-bitrate", 0,
               "max-lateness", -1,
               nullptr);

  gst_bin_add_many(GST_BIN(pipeline), appsrc, parse, queue, pay, sink, nullptr);
  if (!gst_element_link_many(appsrc, parse, queue, pay, sink, nullptr))
  {
    RCLCPP_ERROR(get_logger(), "Failed to link GStreamer elements");
    gst_object_unref(pipeline);
    return false;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE)
  {
    RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
    gst_object_unref(pipeline);
    return false;
  }

  gst_pipeline_ = GST_PIPELINE(pipeline);
  appsrc_ = appsrc;
  h264parse_ = parse;
  rtph264pay_ = pay;
  udpsink_ = sink;
  RCLCPP_INFO(get_logger(), "Direct RTP pipeline started: %s:%d (PT=%u, SSRC=%u, MTU=%d)",
              rtp_host_.c_str(), rtp_port_, rtp_pt_, rtp_ssrc_, rtp_mtu_);
  return true;
}

void LiveviewModule::stop_rtp_pipeline()
{
  if (!gst_pipeline_) return;
  gst_element_set_state(GST_ELEMENT(gst_pipeline_), GST_STATE_NULL);
  gst_object_unref(gst_pipeline_);
  gst_pipeline_ = nullptr;
  appsrc_ = nullptr;
  h264parse_ = nullptr;
  rtph264pay_ = nullptr;
  udpsink_ = nullptr;
}

// Very small NAL scan to detect IDR/SPS/PPS and optionally rebuild Annex-B with only allowed types
bool LiveviewModule::filter_iframes_only(const uint8_t* in, uint32_t len, std::vector<uint8_t>& out)
{
  // Accept both Annex-B and AVCC; rebuild Annex-B with only {5,6,7,8,9}
  const uint8_t* p = in; const uint8_t* end = in + len;
  auto push_sc = [&out]() { const uint8_t sc[4] = {0,0,0,1}; out.insert(out.end(), sc, sc+4); };
  bool wrote = false;

  // First detect Annex-B start codes quickly
  bool saw_start = false;
  for (size_t i = 0; i + 3 < len; ++i) {
    if ((in[i] == 0 && in[i+1] == 0 && in[i+2] == 1) ||
        (i + 4 < len && in[i] == 0 && in[i+1] == 0 && in[i+2] == 0 && in[i+3] == 1)) {
      saw_start = true; break;
    }
  }

  if (saw_start) {
    // Annex-B: iterate start-code delimited units
    size_t i = 0;
    while (i + 3 < len) {
      size_t sc_len = 0;
      if (i + 4 <= len && in[i]==0 && in[i+1]==0 && in[i+2]==0 && in[i+3]==1) { sc_len = 4; }
      else if (i + 3 <= len && in[i]==0 && in[i+1]==0 && in[i+2]==1) { sc_len = 3; }
      if (!sc_len) { ++i; continue; }
      size_t nal_start = i + sc_len;
      size_t j = nal_start;
      while (j + 3 < len && !(in[j]==0 && in[j+1]==0 && ((in[j+2]==1) || (j+3<len && in[j+2]==0 && in[j+3]==1)))) {
        ++j;
      }
      if (nal_start < len) {
        uint8_t nal_type = in[nal_start] & 0x1F;
        if (nal_type==5 || nal_type==6 || nal_type==7 || nal_type==8 || nal_type==9) {
          push_sc(); out.insert(out.end(), in+nal_start, in+std::min(j, (size_t)len)); wrote = true;
        }
      }
      i = j;
    }
    return wrote;
  } else {
    // AVCC: length-prefixed (4 bytes)
    size_t i = 0;
    while (i + 4 <= len) {
      uint32_t n = (in[i]<<24) | (in[i+1]<<16) | (in[i+2]<<8) | (in[i+3]);
      if (n == 0 || i + 4 + n > len) break;
      size_t start = i + 4;
      uint8_t nal_type = in[start] & 0x1F;
      if (nal_type==5 || nal_type==6 || nal_type==7 || nal_type==8 || nal_type==9) {
        push_sc(); out.insert(out.end(), in+start, in+start+n); wrote = true;
      }
      i += 4 + n;
    }
    return wrote;
  }
}

bool LiveviewModule::push_h264_to_pipeline(const uint8_t* buffer, uint32_t buffer_length)
{
  if (!gst_pipeline_ || !appsrc_) return false;

  std::vector<uint8_t> bytes;
  const uint8_t* data = buffer;
  uint32_t len = buffer_length;
  if (direct_iframes_only_)
  {
    bytes.reserve(len + 64);
    bool ok = filter_iframes_only(buffer, buffer_length, bytes);
    if (!ok) return true; // nothing to push for non-IDR frames
    data = bytes.data();
    len = static_cast<uint32_t>(bytes.size());
  }

  GstBuffer* gstbuf = gst_buffer_new_allocate(nullptr, len, nullptr);
  if (!gstbuf) return false;
  GstMapInfo map;
  if (!gst_buffer_map(gstbuf, &map, GST_MAP_WRITE))
  {
    gst_buffer_unref(gstbuf);
    return false;
  }
  memcpy(map.data, data, len);
  gst_buffer_unmap(gstbuf, &map);

  // Timestamp buffer with current clock; we don't have per-frame timestamps from DJI API here
  GstClockTime now = gst_util_uint64_scale(this->get_clock()->now().nanoseconds(), 1, 1);
  GST_BUFFER_PTS(gstbuf) = now;
  GST_BUFFER_DTS(gstbuf) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(gstbuf) = GST_CLOCK_TIME_NONE;

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), gstbuf);
  if (ret != GST_FLOW_OK)
  {
    RCLCPP_DEBUG(get_logger(), "appsrc push returned %d", ret);
    return false;
  }
  return true;
}
void
LiveviewModule::auto_request_keyframe_callback()
{
  // Only request keyframes if module is initialized AND streaming is active
  if (!is_module_initialized_ || !is_streaming_active_)
  {
    return;
  }

  // Request keyframe for the currently active camera stream
  T_DjiReturnCode return_code = DjiLiveview_RequestIntraframeFrameData(
      payload_index_, selected_camera_source_);
  
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_DEBUG(get_logger(),
                 "Auto keyframe request failed for payload=%d, source=%d, error=%ld",
                 payload_index_, selected_camera_source_, return_code);
  }
  else
  {
    RCLCPP_DEBUG(get_logger(),
                 "Auto keyframe requested for payload=%d, source=%d",
                 payload_index_, selected_camera_source_);
  }
}

}  // namespace psdk_ros2
