/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "ffmpeg_image_transport/ffmpeg_publisher.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace ffmpeg_image_transport {


  void FFMPEGPublisher::packetReady(const FFMPEGPacketConstPtr &pkt) {
    (*publishFunction_)(*pkt);
  }

  static bool is_equal(const EncoderConfig &a,
                       const EncoderConfig &b) {
    return (a.encoder  == b.encoder &&
            a.profile  == b.profile &&
            a.qmax     == b.qmax &&
            a.bit_rate == b.bit_rate &&
            a.gop_size == b.gop_size &&
            a.measure_performance == b.measure_performance);
  }
  
  void
  FFMPEGPublisher::configure(EncoderConfig& config, int level) {
    if (!is_equal(config_, config)) {
      config_ = config;
      setCodecFromConfig(config);
      encoder_->reset(); // will be opened on next image
    }
  }


  void FFMPEGPublisher::advertiseImpl(
      rclcpp::Node * node, const std::string & base_topic,
          rmw_qos_profile_t custom_qos) {
    const std::string transportTopic =  getTopicToAdvertise(base_topic);
    this->nh_ = node;
    //nh_.reset(new ros::NodeHandle(transportTopic));
    initConfigServer();
    // make the queue twice the size between keyframes.
    auto queue_size = std::max((int)custom_qos.depth, 2 * config_.gop_size);
    rmw_qos_profile_t qos1 = custom_qos;
    qos1.depth = queue_size;
    this->encoder_ = std::make_shared<FFMPEGEncoder>(node->get_logger());
    FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos1);
  }

  void FFMPEGPublisher::setCodecFromConfig(const EncoderConfig &config) {
    encoder_->setCodec(config.encoder);
    encoder_->setProfile(config.profile);
    encoder_->setPreset(config.preset);
    encoder_->setQMax(config.qmax);
    encoder_->setBitRate(config.bit_rate);
    encoder_->setGOPSize(config.gop_size);
    encoder_->setMeasurePerformance(config.measure_performance);
    std::stringstream ss;
    ss << "FFMPEGPublisher codec: " << config.encoder <<
                    ", profile: " << config.profile <<
                    ", preset: " << config.preset <<
                    ", bit rate: " << config.bit_rate <<
                    ", qmax: " << config.qmax;
    RCLCPP_DEBUG(nh_->get_logger(), ss.str());
  }


  void
  FFMPEGPublisher::publish(const sensor_msgs::msg::Image& message,
                           const PublishFn &publish_fn) const {
    FFMPEGPublisher *me = const_cast<FFMPEGPublisher *>(this);
    if (!me->encoder_->isInitialized()) {
      me->initConfigServer();
      me->publishFunction_ = &publish_fn;
      if (!me->encoder_->initialize(message.width, message.height,
              std::bind(&FFMPEGPublisher::packetReady, me, std::placeholders::_1))) {
        RCLCPP_ERROR(me->nh_->get_logger(),"cannot initialize encoder!");
        return;
      }
    }
    me->encoder_->encodeImage(message); // may trigger packetReady() callback(s) from encoder!
    Lock lock(me->configMutex_);
    if (me->config_.measure_performance) {
      if (++me->frameCounter_ > (unsigned int)me->config_.performance_interval) {
        me->encoder_->printTimers(nh_->get_namespace());
        me->encoder_->resetTimers();
        me->frameCounter_ = 0;
      }
    }
  }
  
}
