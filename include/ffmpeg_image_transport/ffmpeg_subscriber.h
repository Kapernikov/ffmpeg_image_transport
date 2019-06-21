/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <image_transport/simple_subscriber_plugin.h>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace ffmpeg_image_transport {
  using Image         = sensor_msgs::msg::Image;
  using ImagePtr      = sensor_msgs::msg::Image::SharedPtr;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
  
  typedef image_transport::SimpleSubscriberPlugin<
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket>  FFMPEGSubscriberPlugin;
  class FFMPEGSubscriber: public FFMPEGSubscriberPlugin  {
  public:
    virtual ~FFMPEGSubscriber() {}

    virtual std::string getTransportName() const {
      return "ffmpeg";
    }

  protected:
    virtual void
    internalCallback(const typename FFMPEGPacket::ConstSharedPtr& message,
                     const Callback& user_cb) override;

    virtual void subscribeImpl(
        rclcpp::Node * node,
        const std::string & base_topic,
        const Callback & callback,
        rmw_qos_profile_t custom_qos) override;

    /*virtual void
    subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic,
                  uint32_t queue_size, const Callback &callback,
                  const ros::VoidPtr &tracked_object,
                  const image_transport::TransportHints &transport_hints)
      override;*/
  private:
    void frameReady(const ImageConstPtr &img, bool isKeyFrame) const;
    std::shared_ptr<FFMPEGDecoder> decoder_;
    std::string   decoderType_;
    rclcpp::Node *node;
    const Callback *userCallback_;
  };
}
