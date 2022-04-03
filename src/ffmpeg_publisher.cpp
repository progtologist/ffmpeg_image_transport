#include "ffmpeg_image_transport/ffmpeg_publisher.hpp"
#include "sensor_msgs/image_encodings.hpp"

extern "C"
{
  #include <libavcodec/avcodec.h>
  #include <libavformat/avformat.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
  #include <libavutil/opt.h>
}

namespace
{
char ERROR_STRING[AV_ERROR_MAX_STRING_SIZE] = {0};
const char * getErrorStr(const int & error)
{
  memset(ERROR_STRING, 0, sizeof(ERROR_STRING));
  av_make_error_string(ERROR_STRING, AV_ERROR_MAX_STRING_SIZE, error);
  return ERROR_STRING;
}
}

namespace ffmpeg_image_transport
{

struct FFMPEGPublisher::FFMPEGCodec
{
  const AVCodec * codec_ = nullptr;
  AVCodecContext * codec_ctx_ = nullptr;
  AVFrame * frame_ = nullptr;
  SwsContext * sws_ctx_ = nullptr;
  AVPacket * packet_ = nullptr;
};

FFMPEGPublisher::FFMPEGPublisher()
: codec_(new FFMPEGCodec)
{
}

FFMPEGPublisher::~FFMPEGPublisher()
{
  cleanup();
}

std::string FFMPEGPublisher::getTransportName() const
{
  return "ffmpeg";
}

void FFMPEGPublisher::advertiseImpl(
  rclcpp::Node * nh, const std::string & base_topic,
  rmw_qos_profile_t custom_qos)
{
  node_clock_ = nh->get_node_clock_interface();
  node_logging_ = nh->get_node_logging_interface();
  node_params_ = nh->get_node_parameters_interface();

  if (!node_params_->has_parameter("camera_fps_parameter")) {
    node_params_->declare_parameter("camera_fps_parameter", rclcpp::ParameterValue("camera.fps"));
  }
  if (!node_params_->has_parameter("codec_name")) {
    node_params_->declare_parameter("codec_name", rclcpp::ParameterValue("libx264"));
  }
  if (!node_params_->has_parameter("Bitrate")) {
    node_params_->declare_parameter("Bitrate", rclcpp::ParameterValue(512000));
  }
  if (!node_params_->has_parameter("Maximum quantizer")) {
    node_params_->declare_parameter("Maximum quantizer", rclcpp::ParameterValue(51));
  }

  image_transport::SimplePublisherPlugin<CompressedImage>::advertiseImpl(
    nh, base_topic,
    custom_qos);
}

void FFMPEGPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  int error;
  auto new_settings = getSettings(message);
  if (settings_ != new_settings) {
    cleanup();
    if (!initialize(new_settings)) {
      return;
    }
    settings_ = new_settings;
  }

  // Convert message using the software scaler
  unsigned char * ptr[1];
  ptr[0] = (unsigned char *) &message.data[0];
  int srcstride = message.step;

  sws_scale(
    codec_->sws_ctx_, ptr, &srcstride, 0, settings_.height, codec_->frame_->data,
    codec_->frame_->linesize);

  // Create the Packet
  codec_->packet_ = av_packet_alloc();
  if (!codec_->packet_) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Unable to create new packet %s", getErrorStr(error));
    return;
  }

  // Send the frame to the encoder
  error = avcodec_send_frame(codec_->codec_ctx_, codec_->frame_);
  if (error != 0) {
    RCLCPP_WARN(
      node_logging_->get_logger(), "Unable to send frame to codec context %s",
      getErrorStr(error));
    return;
  }
  // Get the packet from the encoder
  error = avcodec_receive_packet(codec_->codec_ctx_, codec_->packet_);
  if (error != 0) {
    RCLCPP_WARN(
      node_logging_->get_logger(), "Unable to receive packet from codec context %s",
      getErrorStr(error));
    return;
  }

  CompressedImage image;
  image.header = message.header;
  image.data.resize(codec_->packet_->size);
  image.encoding = message.encoding;
  image.width = message.width;
  image.height = message.height;
  image.fps = settings_.fps;
  memcpy(&image.data[0], codec_->packet_->data, codec_->packet_->size);
  image.codec = codec_->codec_->id;
  publish_fn(image);
  av_packet_free(&codec_->packet_);
}

bool FFMPEGPublisher::initialize(const EncoderSettings & settings) const
{
  int error;
  // Allocate Codec
  codec_->codec_ = avcodec_find_encoder_by_name(settings.codec_name.c_str());
  if (!codec_->codec_) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Unable to find codec " << settings.codec_name);
    return false;
  }
  if (codec_->codec_->type != AVMEDIA_TYPE_VIDEO) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Codec specified is not a Video Codec " << settings.codec_name);
    cleanup();
    return false;
  }

  // Allocate Codec Context
  codec_->codec_ctx_ = avcodec_alloc_context3(codec_->codec_);
  if (!codec_->codec_ctx_) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Unable to allocate context for codec " << settings.codec_name);
    cleanup();
    return false;
  }

  codec_->codec_ctx_->bit_rate =
    node_params_->get_parameter("Bitrate").as_int();
  codec_->codec_ctx_->qmax = node_params_->get_parameter("Maximum Quantizer").as_int();
  codec_->codec_ctx_->height = settings.height;
  codec_->codec_ctx_->width = settings.width;
  codec_->codec_ctx_->time_base.num = 1;
  codec_->codec_ctx_->time_base.den = settings.fps;
  codec_->codec_ctx_->gop_size = settings.fps / 2;
  codec_->codec_ctx_->max_b_frames = 2;
  codec_->codec_ctx_->pix_fmt = AVPixelFormat::AV_PIX_FMT_YUV420P;
  av_opt_set(codec_->codec_ctx_->priv_data, "profile", "main", AV_OPT_SEARCH_CHILDREN);
  av_opt_set(codec_->codec_ctx_->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
  av_opt_set(codec_->codec_ctx_->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
  // Open Codec Context
  error = avcodec_open2(codec_->codec_ctx_, codec_->codec_, NULL);
  if (error < 0) {
    RCLCPP_ERROR(node_logging_->get_logger(), "Could not open the encoder %s", getErrorStr(error));
    cleanup();
    return false;
  }
  // Allocate Frame
  codec_->frame_ = av_frame_alloc();
  if (!codec_->frame_) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(), "Unable to allocate frame for codec " << settings.codec_name);
    cleanup();
    return false;
  }
  codec_->frame_->width = settings.width;
  codec_->frame_->height = settings.height;

  try {
    initSWS(settings.encoding);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      e.what() << " by codec " << settings.codec_name);
    cleanup();
    return false;
  }

  //Allocate picture region
  error = av_image_alloc(
    codec_->frame_->data, codec_->frame_->linesize, settings.width,
    settings.height, codec_->codec_ctx_->pix_fmt, 1);
  if (error < 0) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Unable to allocate picture region for codec %s",
      getErrorStr(error));
    cleanup();
    return false;
  }

  av_log_set_level(AV_LOG_QUIET); // silence all the errors/warnings
  return true;
}

FFMPEGPublisher::EncoderSettings FFMPEGPublisher::getSettings(
  const sensor_msgs::msg::Image & message)
const
{
  EncoderSettings settings;
  settings.height = message.height;
  settings.width = message.width;
  settings.encoding = message.encoding;
  auto parameter_path = node_params_->get_parameter("Camera FPS Parameter").as_string();
  try {
    auto fps_param = node_params_->get_parameter(parameter_path);
    if (fps_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      settings.fps = int(fps_param.as_double());
    } else if (fps_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      settings.fps = fps_param.as_int();
    } else {
      std::stringstream ss;
      ss << "Parameter type " << fps_param.get_type_name() <<
        " not supported for parameter path " <<
        parameter_path;
      throw std::runtime_error(ss.str());
    }
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_WARN_THROTTLE(
      node_logging_->get_logger(),
      *node_clock_->get_clock(), 10000,
      "Could not get framerate from camera node, defaulting to 10fps");
    settings.fps = 10;
  }
  settings.codec_name = node_params_->get_parameter("Codec Name").as_string();
  return settings;
}

void FFMPEGPublisher::cleanup() const
{
  // Memory Cleanup
  if (codec_->codec_ctx_) {
    avcodec_free_context(&codec_->codec_ctx_);
  }
  if (codec_->frame_) {
    av_frame_free(&codec_->frame_);
  }
  if (codec_->sws_ctx_) {
    sws_freeContext(codec_->sws_ctx_);
  }
  if (codec_->packet_) {
    av_packet_free(&codec_->packet_);
  }
}

void FFMPEGPublisher::initSWS(const std::string & encoding) const
{
  using namespace sensor_msgs::image_encodings;
  int & src_fmt = codec_->frame_->format;
  const int & width = codec_->codec_ctx_->width;
  const int & height = codec_->codec_ctx_->height;
  const AVPixelFormat & dst_fmt = codec_->codec_ctx_->pix_fmt;

  // Prepare the software scale context
  // Will convert from RGB24 to YUV420P
  if (encoding == BGR8) {
    src_fmt = AV_PIX_FMT_BGR24;
  } else if (encoding == RGB8) {
    src_fmt = AV_PIX_FMT_RGB24;
  } else if (encoding == RGB16) {
    src_fmt = AV_PIX_FMT_RGB48;
  } else if (encoding == YUV422) {
    src_fmt = AV_PIX_FMT_UYVY422;
  } else if (encoding == BAYER_GBRG16) {
    src_fmt = AV_PIX_FMT_BAYER_GBRG16LE;
  } else {
    std::stringstream ss;
    ss << "Encoding " << encoding << " not supported";
    throw std::runtime_error(ss.str());
  }
  codec_->sws_ctx_ = sws_getContext(
    width, height, AVPixelFormat(src_fmt),  //src
    width, height, dst_fmt,                 //dest
    SWS_FAST_BILINEAR, NULL, NULL, NULL);
}

}
