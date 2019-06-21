/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace ffmpeg_image_transport {

  void FFMPEGSubscriber::frameReady(const ImageConstPtr &img,
                                    bool) const {
    (*userCallback_)(img);
  }


  void FFMPEGSubscriber::subscribeImpl(
      rclcpp::Node * node,
      const std::string & base_topic,
      const Callback & callback,
          rmw_qos_profile_t custom_qos) {
    // bump queue size a bit to avoid lost packets
    this->node = node;
    this->decoderType_ = node->declare_parameter("decoder_type",rclcpp::ParameterValue("")).get<std::string>();
    auto queue_size = std::max((int)custom_qos.depth, 20);
    auto qos1 = custom_qos;
    qos1.depth = queue_size;
    this->decoder_ = std::make_shared<FFMPEGDecoder>(node->get_logger());
    FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic,
                                          callback, qos1
                                          );
  }

  void FFMPEGSubscriber::internalCallback(const FFMPEGPacket::ConstSharedPtr& msg,
                                        const Callback& user_cb) {
    if (!decoder_->isInitialized()) {
      if (msg->flags == 0) {
        return; // wait for key frame!
      }
      userCallback_ = &user_cb;
      if (!decoder_->initialize(
            msg, std::bind(&FFMPEGSubscriber::frameReady, this, std::placeholders::_1, std::placeholders::_2),
            decoderType_)) {
        RCLCPP_ERROR(node->get_logger(),"cannot initialize decoder!");
        return;
      }
    }
    decoder_->decodePacket(msg);
  }
}
