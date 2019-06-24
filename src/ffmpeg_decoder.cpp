/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace ffmpeg_image_transport {

  FFMPEGDecoder::FFMPEGDecoder(rclcpp::Logger l) : logger(l) {
    codecMap_["h264_nvenc"] = {"h264"};
    codecMap_["libx264"]    = {"h264"};
    codecMap_["mjpeg"]    = {"mjpeg"};
    codecMap_["hevc_nvenc"] = {"hevc_cuvid", "hevc"};
  }

  FFMPEGDecoder::~FFMPEGDecoder() {
    reset();
  }

  void FFMPEGDecoder::reset() {
    if (codecContext_) {
      avcodec_close(codecContext_);
      av_free(codecContext_);
      codecContext_ = NULL;
    }
    if (swsContext_) {
      sws_freeContext(swsContext_);
      swsContext_ = NULL;
    }
    av_free(decodedFrame_);
    decodedFrame_ = NULL;
    av_free(colorFrame_);
    colorFrame_ = NULL;
  }

  bool FFMPEGDecoder::initialize(const FFMPEGPacket::ConstSharedPtr& msg,
                                 Callback callback, const std::string &codecName) {
    callback_ = callback;
    std::string cname = codecName;
    std::vector<std::string> codecs;
    if (cname.empty()) {
      // try and find the right codec from the map
      const auto it = codecMap_.find(msg->encoding);
      if (it == codecMap_.end()) {
        RCLCPP_ERROR(logger, "unknown encoding: %s", msg->encoding.c_str());
        return (false);
      }
      cname  = msg->encoding;
      codecs = it->second;
    } else {
      codecs.push_back(codecName);
    }
    encoding_ = msg->encoding;
    return (initDecoder(msg->img_width, msg->img_height, cname, codecs));
  }

	bool FFMPEGDecoder::initDecoder(int width, int height,
                                  const std::string &codecName,
                                  const std::vector<std::string> &codecs) {
    try {
      const AVCodec *codec = NULL;
      for (const auto &c: codecs) {
        codec = avcodec_find_decoder_by_name(c.c_str());
        if (!codec) {
          RCLCPP_WARN(logger, "no codec %s found", c.c_str());
          continue;
        }
        codecContext_ = avcodec_alloc_context3(codec);
        if (!codecContext_) {
          RCLCPP_WARN(logger,"alloc context failed for %s", codecName.c_str());
          codec = NULL;
          continue;
        }
        codecContext_->width  = width;
        codecContext_->height = height;
        if (avcodec_open2(codecContext_, codec, NULL) < 0) {
          RCLCPP_WARN(logger, "open context failed for ", codecName.c_str());
          av_free(codecContext_);
          codecContext_ = NULL;
          codec = NULL;
          continue;
        }
      }
      if (!codec)
        throw (std::runtime_error("cannot open codec " + codecName));
      
      decodedFrame_       = av_frame_alloc();
      colorFrame_         = av_frame_alloc();
      colorFrame_->width  = width;
      colorFrame_->height = height;
      colorFrame_->format = AV_PIX_FMT_BGR24;


    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(logger, e.what());
      reset();
      return (false);
    }
    RCLCPP_INFO(logger, "using decoder %s", codecName.c_str());
    return (true);
	}

  bool FFMPEGDecoder::decodePacket(const FFMPEGPacket::ConstSharedPtr &msg) {
    rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
    rclcpp::Time t0;
    if (measurePerformance_) {
      t0 = ros_clock.now();
    }
    if (msg->encoding != encoding_) {
      RCLCPP_ERROR(logger,"cannot change encoding on the fly!!!");
      return (false);
    }
    AVCodecContext *ctx = codecContext_;
    AVPacket packet;
    av_init_packet(&packet);
    av_new_packet(&packet, msg->data.size()); // will add some padding!
    memcpy(packet.data, &msg->data[0], msg->data.size());
    packet.pts = msg->pts;
    packet.dts = packet.pts;
    ptsToStamp_[packet.pts] = msg->header.stamp;
    int ret = avcodec_send_packet(ctx, &packet);
    if (ret != 0) {
      RCLCPP_WARN(logger, "send_packet failed for pts: %i",  msg->pts);
      av_packet_unref(&packet);
      return (false);
    }
    ret = avcodec_receive_frame(ctx, decodedFrame_);
    if (ret == 0 && decodedFrame_->width != 0) {
      // convert image to something palatable
      if (!swsContext_) {
        swsContext_ = sws_getContext(
          ctx->width, ctx->height, (AVPixelFormat)decodedFrame_->format, //src
          ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format, // dest
          SWS_FAST_BILINEAR, NULL, NULL, NULL);
        if (!swsContext_) {
          RCLCPP_ERROR(logger,"cannot allocate sws context!!!!");
          rclcpp::shutdown();
          return (false);
        }
      }
      // prepare the decoded message
      ImagePtr image(new sensor_msgs::msg::Image());
      image->height = decodedFrame_->height;
      image->width  = decodedFrame_->width;
      image->step   = image->width * 3; // 3 bytes per pixel
      image->encoding = sensor_msgs::image_encodings::BGR8;
      image->data.resize(image->step * image->height);

      // bend the memory pointers in colorFrame to the right locations 
      av_image_fill_arrays(colorFrame_->data,  colorFrame_->linesize,
                           &(image->data[0]),
                           (AVPixelFormat)colorFrame_->format,
                           colorFrame_->width, colorFrame_->height, 1);
      sws_scale(swsContext_,
                decodedFrame_->data,  decodedFrame_->linesize, 0, // src
                ctx->height, colorFrame_->data, colorFrame_->linesize); // dest
      auto it = ptsToStamp_.find(decodedFrame_->pts);
      if (it == ptsToStamp_.end()) {
        RCLCPP_ERROR(logger,"cannot find pts that matches %i",
                          decodedFrame_->pts);
      } else {
        image->header = msg->header;
        image->header.stamp = it->second;
        ptsToStamp_.erase(it);
        callback_(image, decodedFrame_->key_frame == 1); // deliver     callback
      }
    }
    av_packet_unref(&packet);
    if (measurePerformance_) {
      rclcpp::Time t1 = ros_clock.now();
      double dt = (t1-t0).seconds();
      tdiffTotal_.update(dt);
    }
    return (true);
  }

  void FFMPEGDecoder::resetTimers() {
    tdiffTotal_.reset();
  }

  void FFMPEGDecoder::printTimers(const std::string &prefix) const {
    RCLCPP_INFO(logger, "%s total decode: ", prefix.c_str(), tdiffTotal_.to_string().c_str());
  }
 
}  // namespace
