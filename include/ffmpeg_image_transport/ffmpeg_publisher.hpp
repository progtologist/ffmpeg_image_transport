#ifndef FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
#define FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_

#include <memory>

#include "image_transport/simple_publisher_plugin.hpp"
#include "ffmpeg_image_transport/msg/packet.hpp"

namespace ffmpeg_image_transport
{
using CompressedImage = ffmpeg_image_transport::msg::Packet;

class FFMPEGPublisher : public image_transport::SimplePublisherPlugin<CompressedImage>
{
public:
  FFMPEGPublisher();

  virtual ~FFMPEGPublisher();

  std::string getTransportName() const;

protected:
  struct FFMPEGCodec;
  struct EncoderSettings
  {
    bool operator==(const EncoderSettings & other) const
    {
      if (height == other.height &&
        width == other.width &&
        fps == other.fps &&
        codec_name == other.codec_name &&
        encoding == other.encoding)
      {
        return true;
      }
      return false;
    }
    bool operator!=(const EncoderSettings & other) const
    {
      return !(*this == other);
    }

    std::string codec_name;
    std::string encoding;
    int height;
    int width;
    int fps;
  };

  void advertiseImpl(
    rclcpp::Node * nh, const std::string & base_topic,
    rmw_qos_profile_t custom_qos);

  void publish(const sensor_msgs::msg::Image & message, const PublishFn & publish_fn) const;

  bool initialize(const EncoderSettings & settings) const;

  EncoderSettings getSettings(const sensor_msgs::msg::Image & message) const;

  void cleanup() const;

  void initSWS(const std::string & encoding) const;

  std::unique_ptr<FFMPEGCodec> codec_;
  mutable EncoderSettings settings_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
};

}

#endif  // FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
