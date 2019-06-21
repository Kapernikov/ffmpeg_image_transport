/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "ffmpeg_image_transport/ffmpeg_encoder.h"
//#include "ffmpeg_image_transport/EncoderDynConfig.h"

#include <image_transport/simple_publisher_plugin.h>
//#include <dynamic_reconfigure/server.h>

#include <mutex>
#include <memory>

struct EncoderConfig{
    std::string encoder;
    std::string profile;
    std::string preset;
    int qmax;
    int bit_rate;
    int gop_size;
    bool measure_performance;
    int performance_interval;
};

namespace ffmpeg_image_transport {
  typedef image_transport::SimplePublisherPlugin<
    ffmpeg_image_transport_msgs::msg::FFMPEGPacket>  FFMPEGPublisherPlugin;

  class FFMPEGPublisher : public FFMPEGPublisherPlugin {
    typedef std::unique_lock<std::recursive_mutex> Lock;
    using FFMPEGPacketConstPtr = ffmpeg_image_transport_msgs::msg::FFMPEGPacket::ConstPtr;
  public:
    virtual std::string getTransportName() const override {
      return "ffmpeg";
    }
    void configure(EncoderConfig& config, int level);

  protected:
    // override to set up reconfigure server
    virtual void advertiseImpl(
        rclcpp::Node * node, const std::string & base_topic,
        rmw_qos_profile_t custom_qos) override;

    void publish(const sensor_msgs::msg::Image& message,
                 const PublishFn& publish_fn) const override;

  private:
    void packetReady(const FFMPEGPacketConstPtr &pkt);
    void setCodecFromConfig(const EncoderConfig &cfg);
    void initializeParameters();
    // variables ---------
    rclcpp::Node *    nh_;
    const PublishFn              *publishFunction_{NULL};
    std::shared_ptr<FFMPEGEncoder>                 encoder_;
    unsigned int                  frameCounter_{0};
    EncoderConfig              config_;
    std::recursive_mutex          configMutex_;
  };
}
