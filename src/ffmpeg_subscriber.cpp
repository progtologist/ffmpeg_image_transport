#include "ffmpeg_image_transport/ffmpeg_subscriber.hpp"
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

struct FFMPEGSubscriber::FFMPEGCodec
{
  const AVCodec * codec_ = nullptr;
  AVCodecContext * codec_ctx_ = nullptr;
  AVFrame * frame_ = nullptr;
  SwsContext * sws_ctx_ = nullptr;
  AVPacket * packet_ = nullptr;
};

FFMPEGSubscriber::FFMPEGSubscriber()
: codec_(new FFMPEGCodec)
{
}

FFMPEGSubscriber::~FFMPEGSubscriber()
{
  cleanup();
}

std::string FFMPEGSubscriber::getTransportName() const
{
  return "ffmpeg";
}

void FFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * nh, const std::string & base_topic,
  const image_transport::SubscriberPlugin::Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  node_clock_ = nh->get_node_clock_interface();
  node_logging_ = nh->get_node_logging_interface();
  node_params_ = nh->get_node_parameters_interface();

  image_transport::SimpleSubscriberPlugin<CompressedImage>::subscribeImpl(
    nh, base_topic, callback,
    custom_qos);
}

void FFMPEGSubscriber::internalCallback(
  const CompressedImage::ConstSharedPtr & message,
  const image_transport::SubscriberPlugin::Callback & user_cb)
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
  // Create the Packet
  codec_->packet_ = av_packet_alloc();
  if (!codec_->packet_) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Unable to create new packet %s", getErrorStr(error));
    return;
  }
  error = av_new_packet(codec_->packet_, message->data.size());
  if (error != 0) {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Unable to allocate packet to codec context %s",
      getErrorStr(error));
    return;
  }

  memcpy(codec_->packet_->data, &message->data[0], message->data.size());

  // Send the packet to the decoder
  error = avcodec_send_packet(codec_->codec_ctx_, codec_->packet_);
  if (error != 0) {
    RCLCPP_WARN(
      node_logging_->get_logger(), "Unable to send packet to codec context %s",
      getErrorStr(error));
    return;
  }

  // Get the frame from the decoder
  error = avcodec_receive_frame(codec_->codec_ctx_, codec_->frame_);
  if (error != 0) {
    RCLCPP_WARN(
      node_logging_->get_logger(), "Unable to receive frame from codec context %s",
      getErrorStr(error));
    return;
  }

  // Create (and allocate) the resulting message
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->header = message->header;
  image->width = message->width;
  image->height = message->height;
  image->step = image->width * sensor_msgs::image_encodings::numChannels(message->encoding);
  image->encoding = message->encoding;
  image->data.resize(image->step * image->height);
  image->is_bigendian = false;

  // Convert the message using the software scaler
  unsigned char * ptr[1];
  ptr[0] = (unsigned char *) &image->data[0];
  int dststride = image->step;

  sws_scale(
    codec_->sws_ctx_, codec_->frame_->data, codec_->frame_->linesize, 0, settings_.height, ptr,
    &dststride);
  user_cb(image);
  av_packet_free(&codec_->packet_);
}

bool FFMPEGSubscriber::initialize(const DecoderSettings & settings) const
{
  int error;
  // Allocate Codec
  codec_->codec_ = avcodec_find_decoder(AVCodecID(settings.codec));
  if (!codec_->codec_) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Unable to find codec " << std::to_string(settings.codec) << " failing!");
    cleanup();
    return false;
  }
  // Allocate Codec Context
  codec_->codec_ctx_ = avcodec_alloc_context3(codec_->codec_);
  if (!codec_->codec_ctx_) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Unable to allocate context for codec " << std::to_string(settings.codec));
    cleanup();
    return false;
  }
  codec_->codec_ctx_->height = settings.height;
  codec_->codec_ctx_->width = settings.width;
  codec_->codec_ctx_->pix_fmt = AVPixelFormat::AV_PIX_FMT_YUV420P;
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
      node_logging_->get_logger(),
      "Unable to allocate frame for codec " << std::to_string(settings.codec));
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
      e.what() << " by codec " << std::to_string(settings.codec));
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

FFMPEGSubscriber::DecoderSettings FFMPEGSubscriber::getSettings(
  const CompressedImage::ConstSharedPtr & message) const
{
  DecoderSettings settings;
  settings.height = message->height;
  settings.width = message->width;
  settings.encoding = message->encoding;
  settings.fps = message->fps;
  settings.codec = message->codec;
  return settings;
}

void FFMPEGSubscriber::cleanup() const
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

void FFMPEGSubscriber::initSWS(const std::string & encoding) const
{
  using namespace sensor_msgs::image_encodings;
  const AVPixelFormat & src_fmt = codec_->codec_ctx_->pix_fmt;
  const int & width = codec_->codec_ctx_->width;
  const int & height = codec_->codec_ctx_->height;
  int & dst_fmt = codec_->frame_->format;

  // Prepare the software scale context
  // Will convert from YUV420P to RGB24
  if (encoding == BGR8) {
    dst_fmt = AV_PIX_FMT_BGR24;
  } else if (encoding == RGB8) {
    dst_fmt = AV_PIX_FMT_RGB24;
  } else if (encoding == RGB16) {
    dst_fmt = AV_PIX_FMT_RGB48;
  } else if (encoding == YUV422) {
    dst_fmt = AV_PIX_FMT_UYVY422;
  } else if (encoding == BAYER_GBRG16) {
    dst_fmt = AV_PIX_FMT_BAYER_GBRG16LE;
  } else {
    std::stringstream ss;
    ss << "Encoding " << encoding << " not supported";
    throw std::runtime_error(ss.str());
  }
  codec_->sws_ctx_ = sws_getContext(
    width, height, src_fmt,                //src
    width, height, AVPixelFormat(dst_fmt), //dest
    SWS_FAST_BILINEAR, NULL, NULL, NULL);
}

}
