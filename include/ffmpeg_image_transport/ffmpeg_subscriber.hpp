#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_

#include <memory>

#include "image_transport/simple_subscriber_plugin.hpp"
#include "ffmpeg_image_transport/msg/packet.hpp"

namespace ffmpeg_image_transport
{
using CompressedImage = ffmpeg_image_transport::msg::Packet;

class FFMPEGSubscriber : public image_transport::SimpleSubscriberPlugin<CompressedImage>
{
public:
  FFMPEGSubscriber();

  virtual ~FFMPEGSubscriber();

  std::string getTransportName() const;

protected:
  struct FFMPEGCodec;
  struct DecoderSettings
  {
    bool operator==(const DecoderSettings & other) const
    {
      if (height == other.height &&
        width == other.width &&
        fps == other.fps &&
        codec == other.codec &&
        encoding == other.encoding)
      {
        return true;
      }
      return false;
    }
    bool operator!=(const DecoderSettings & other) const
    {
      return !(*this == other);
    }

    int codec;
    std::string encoding;
    int height;
    int width;
    int fps;
  };

  void subscribeImpl(
    rclcpp::Node * nh, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos);

  void internalCallback(const CompressedImage::ConstSharedPtr & message, const Callback & user_cb);

  bool initialize(const DecoderSettings & settings) const;

  DecoderSettings getSettings(const CompressedImage::ConstSharedPtr & message) const;

  void cleanup() const;

  void initSWS(const std::string & encoding) const;

  std::unique_ptr<FFMPEGCodec> codec_;
  mutable DecoderSettings settings_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
};

}

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_SUBSCRIBER_HPP_
