#include "pluginlib/class_list_macros.hpp"
#include "ffmpeg_image_transport/ffmpeg_publisher.hpp"
#include "ffmpeg_image_transport/ffmpeg_subscriber.hpp"

PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::FFMPEGPublisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::FFMPEGSubscriber, image_transport::SubscriberPlugin)
