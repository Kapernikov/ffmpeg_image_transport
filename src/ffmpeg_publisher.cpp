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
FFMPEGPublisher::configure(EncoderConfig& config) {
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
    // make the queue twice the size between keyframes.
    auto queue_size = std::max((int)custom_qos.depth, 2 * config_.gop_size);
    rmw_qos_profile_t qos1 = custom_qos;
    //qos1.depth = queue_size;
    this->encoder_ = std::make_shared<FFMPEGEncoder>(node->get_logger());
    initializeParameters();
    this->setCodecFromConfig(this->config_);
    FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos1);
}

void FFMPEGPublisher::setCodecFromConfig(const EncoderConfig &config) {
    RCLCPP_WARN(nh_->get_logger(), "setting codec to %s", config.encoder.c_str());
    encoder_->setCodec(config.encoder);
    if (config.encoder == "mjpeg") {
        encoder_->setPixFormat(AV_PIX_FMT_YUVJ420P);
    } else {
        encoder_->setPixFormat(AV_PIX_FMT_YUV420P);
    }
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

void FFMPEGPublisher::initializeParameters()
{
    config_.encoder = nh_->declare_parameter("encoder", rclcpp::ParameterValue("libx264")).get<std::string>();

    config_.profile = nh_->declare_parameter("profile", rclcpp::ParameterValue("main")).get<std::string>();
    config_.preset = nh_->declare_parameter("preset", rclcpp::ParameterValue("slow")).get<std::string>();
    config_.qmax = nh_->declare_parameter("qmax", rclcpp::ParameterValue(10)).get<unsigned int>();
    config_.bit_rate = nh_->declare_parameter("bit_rate", rclcpp::ParameterValue(8242880)).get<unsigned int>();
    config_.gop_size = nh_->declare_parameter("gop_size", rclcpp::ParameterValue(15)).get<unsigned int>();
    config_.measure_performance = nh_->declare_parameter("measure_performance", rclcpp::ParameterValue(false)).get<bool>();
    config_.performance_interval =  nh_->declare_parameter("performance_interval", rclcpp::ParameterValue(175)).get<unsigned int>();
    auto existing_callback = nh_->set_on_parameters_set_callback(nullptr);
    nh_->set_on_parameters_set_callback([existing_callback, this](const std::vector<rclcpp::Parameter> &parameters)
                                        -> rcl_interfaces::msg::SetParametersResult {
        auto result = rcl_interfaces::msg::SetParametersResult();
        // first call the existing callback, if there was one
        if (nullptr != existing_callback) {
            result = existing_callback(parameters);
            // if the existing callback failed, go ahead and return the result
            if (!result.successful) {
                return result;
            }

        }
        result.successful = true;
        auto newConfig = config_;
        for (auto parameter: parameters) {
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
                RCLCPP_ERROR(this->nh_->get_logger(), "parameter %s not set", parameter.get_name().c_str());
                result.successful = false;
                return result;
            }
            if (parameter.get_name() == "encoder") {
                newConfig.encoder = parameter.as_string();
            }
            if (parameter.get_name() == "profile") {
                newConfig.profile = parameter.as_string();
            }
            if (parameter.get_name() == "preset") {
                newConfig.preset = parameter.as_string();
            }
            if (parameter.get_name() == "qmax") {
                newConfig.qmax = parameter.as_int();
            }
            if (parameter.get_name() == "bit_rate") {
                newConfig.bit_rate = parameter.as_int();
            }
            if (parameter.get_name() == "gop_size") {
                newConfig.gop_size = parameter.as_int();
            }
            if (parameter.get_name() == "measure_performance") {
                newConfig.measure_performance = parameter.as_bool();
            }
            if (parameter.get_name() == "performance_interval") {
                newConfig.performance_interval = parameter.as_int();
            }
        }
        configure(newConfig);
        return result;

    });
}


void
FFMPEGPublisher::publish(const sensor_msgs::msg::Image& message,
                         const PublishFn &publish_fn) const {
    FFMPEGPublisher *me = const_cast<FFMPEGPublisher *>(this);
    if (!me->encoder_->isInitialized()) {
        RCLCPP_ERROR(me->nh_->get_logger(), "init %i x %i s=%i %s %i", message.width, message.height, message.step, message.encoding.c_str(), message.data.size());
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
