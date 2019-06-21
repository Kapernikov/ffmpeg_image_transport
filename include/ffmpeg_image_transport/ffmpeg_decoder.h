/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "ffmpeg_image_transport/tdiff.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <unordered_map>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avio.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}


namespace ffmpeg_image_transport {
  using Image = sensor_msgs::msg::Image;
  using ImagePtr = sensor_msgs::msg::Image::SharedPtr;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstPtr;
  using FFMPEGPacket = ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
  typedef std::unordered_map<int64_t, rclcpp::Time> PTSMap;

  class FFMPEGDecoder {
  public:
    typedef std::function<void(const ImageConstPtr &img,
                                 bool isKeyFrame)> Callback;
    FFMPEGDecoder(rclcpp::Logger logger);
    ~FFMPEGDecoder();
    bool isInitialized() const { return (codecContext_ != NULL); }
    // Initialize decoder upon first packet received,
    // providing callback to be called when frame is complete.
    // You must still call decodePacket(msg) afterward!
    bool initialize(const FFMPEGPacket::ConstPtr& msg, Callback callback,
                    const std::string &codec=std::string());

    // clears all state, but leaves config intact
    void reset();
    // decode packet (may result in frame callback!)
    bool decodePacket(const FFMPEGPacket::ConstPtr &msg);
    void setMeasurePerformance(bool p) {
      measurePerformance_ = p;
    }
    void printTimers(const std::string &prefix) const;
    void resetTimers();

  private:
    bool initDecoder(int width, int height,
                     const std::string &codecName,
                     const std::vector<std::string> &codec);
    // --------- variables
    Callback          callback_;
    // mapping of header
    PTSMap            ptsToStamp_;
    // performance analysis
    bool              measurePerformance_{false};
    TDiff             tdiffTotal_;
    // libav stuff
    std::string       encoding_;
    rclcpp::Logger   logger;
    AVCodecContext   *codecContext_{NULL};
    AVFrame          *decodedFrame_{NULL};
    AVFrame          *colorFrame_{NULL};
    SwsContext       *swsContext_{NULL};
    std::unordered_map<std::string, std::vector<std::string>> codecMap_;
    AVPacket          packet_;
  };
}
