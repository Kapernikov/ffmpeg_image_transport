/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_encoder.h"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace ffmpeg_image_transport {

  FFMPEGEncoder::FFMPEGEncoder(rclcpp::Logger l): logger(l) {
    // must init the packet and set the pointers to zero
    // in case a closeCodec() happens right away,
    // and av_packet_unref() is called.
    av_init_packet(&packet_);
    packet_.data = NULL;    // packet data will be allocated by the encoder
    packet_.size = 0;
  }

  FFMPEGEncoder::~FFMPEGEncoder() {
    Lock lock(mutex_);
    closeCodec();
  }

  void FFMPEGEncoder::reset() {
    Lock lock(mutex_);
    closeCodec();
  }

  void
  FFMPEGEncoder::closeCodec() {
    if (codecContext_) {
      avcodec_close(codecContext_);
      codecContext_ = NULL;
    }
    if (swsContext_) {
      sws_freeContext(swsContext_);
      swsContext_ = NULL;
    }

    if (frame_) {
      av_free(frame_);
      frame_ = 0;
    }
    if (packet_.data != NULL) {
      av_packet_unref(&packet_); // free packet allocated by encoder
      packet_.data = NULL;
      packet_.size = 0;
    }
  }

  bool FFMPEGEncoder::initialize(int width, int height,
                                 Callback callback) {
    Lock lock(mutex_);
    callback_ = callback;
    return (openCodec(width, height));
  }

	bool FFMPEGEncoder::openCodec(int width, int height) {
    codecContext_ = NULL;
    try {
      if (codecName_.empty()) {
        throw (std::runtime_error("no codec set!"));
      }
      // find codec
      AVCodec *codec = avcodec_find_encoder_by_name(codecName_.c_str());
      if (!codec) {
        throw (std::runtime_error("cannot find codec: " + codecName_));
      }
	    // allocate codec context
      codecContext_ = avcodec_alloc_context3(codec);
      if (!codecContext_) {
        throw (std::runtime_error("cannot allocate codec context!"));
      }
      codecContext_->bit_rate  = bitRate_;
      codecContext_->qmax      = qmax_;// 0: highest, 63: worst quality bound
      codecContext_->width     = width;
      codecContext_->height    = height;
      codecContext_->time_base = timeBase_;
      codecContext_->framerate = frameRate_;

      // gop size is number of frames between keyframes
      // small gop -> higher bandwidth, lower cpu consumption
      codecContext_->gop_size = GOPSize_;
      // number of bidirectional frames (per group?).
      // NVenc can only handle zero!
      codecContext_->max_b_frames = 0;
   
      // encoded pixel format. Must be supported by encoder
      // check with e.g.: ffmpeg -h encoder=h264_nvenc -pix_fmts

      codecContext_->pix_fmt = pixFormat_;




      if (av_opt_set(codecContext_->priv_data, "profile", profile_.c_str(),
                     AV_OPT_SEARCH_CHILDREN) != 0) {
        RCLCPP_ERROR(logger, "cannot set profile: %s", profile_.c_str());
      }

      if (av_opt_set(codecContext_->priv_data, "preset", preset_.c_str(),
                     AV_OPT_SEARCH_CHILDREN) != 0) {
        RCLCPP_ERROR(logger, "cannot set preset: %s", preset_.c_str());
      }
      RCLCPP_DEBUG(logger, "codec: %10s, profile: %10s, preset: %10s,"
                " bit_rate: %10ld qmax: %2d",
                codecName_.c_str(), profile_.c_str(),
                preset_.c_str(), bitRate_, qmax_);
      /* other optimization options for nvenc
         if (av_opt_set_int(codecContext_->priv_data, "surfaces",
         0, AV_OPT_SEARCH_CHILDREN) != 0) {
         ROS_ERROR_STREAM("cannot set surfaces!");
         }
      */
      if (avcodec_open2(codecContext_, codec, NULL) < 0)  {
        throw (std::runtime_error("cannot open codec!"));
      }
      RCLCPP_DEBUG(logger, "opened codec: %s", codecName_.c_str());
      frame_ = av_frame_alloc();
      if (!frame_)  {
        throw (std::runtime_error("cannot alloc frame!"));
      }
      frame_->width  = width;
      frame_->height = height;
      frame_->format = codecContext_->pix_fmt;
      // allocate image for frame
      if (av_image_alloc(frame_->data, frame_->linesize, width, height,
                         (AVPixelFormat)frame_->format, 32) < 0) {
        throw (std::runtime_error("cannot alloc image!"));
      }
      //Initialize packet
      av_init_packet(&packet_);
      packet_.data = NULL;    // packet data will be allocated by the encoder
      packet_.size = 0;
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(logger, e.what());
      if (codecContext_) {
        avcodec_close(codecContext_);
        codecContext_ = NULL;
      }
      if (frame_) {
        av_free(frame_);
        frame_ = 0;
      }
      return (false);
    }
    RCLCPP_DEBUG(logger, "intialized codec %s for image %i x %i", codecName_.c_str(),
                     width , height);
    return (true);
	}

  void FFMPEGEncoder::encodeImage(const sensor_msgs::msg::Image &msg) {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    rclcpp::Time t0;
    if (measurePerformance_) { t0 = clock.now(); }
    cv::Mat img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    encodeImage(img, msg.header, t0);
    if (measurePerformance_) {
      const auto t1 = clock.now();
      tdiffDebayer_.update((t1-t0).seconds());
    }
  }

  void FFMPEGEncoder::encodeImage(const cv::Mat &img,
                                  const std_msgs::msg::Header &header,
                                  const rclcpp::Time &t0) {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    Lock lock(mutex_);
    rclcpp::Time t1, t2, t3;
    if (measurePerformance_) {
      frameCnt_++;
      t1 = clock.now();
      totalInBytes_ += img.cols * img.rows; // raw size!
    }

    const uint8_t *p = img.data;
    const int width  = img.cols;
    const int height = img.rows;
    const AVPixelFormat targetFmt = codecContext_->pix_fmt;

    if (!swsContext_) {
      swsContext_ = sws_getContext(
        width, height, AV_PIX_FMT_BGR24, //src
        width, height, targetFmt, // dest
        SWS_FAST_BILINEAR, NULL, NULL, NULL);
      if (!swsContext_) {
        RCLCPP_ERROR(logger,"cannot allocate sws context!!!!");
        rclcpp::shutdown();
        return;
      }
    }



    if (targetFmt == AV_PIX_FMT_BGR0) {
      memcpy(frame_->data[0], p, width * height * 3);
    } else {
      const int in_linesize[1] = { (int)img.step[0] };
      sws_scale(swsContext_, (const uint8_t * const *)&p, in_linesize, 0,
                height, frame_->data, frame_->linesize);
      //RCLCPP_WARN(logger, "sws: %i %i %i %i %i" , in_linesize[0], frame_->linesize[0],frame_->linesize[1],frame_->linesize[2],frame_->linesize[3]);
    }
    if (measurePerformance_) {
      t2 = clock.now();
      tdiffFrameCopy_.update((t2 - t1).seconds());
    }

    frame_->pts = pts_++; //
    ptsToStamp_.insert(PTSMap::value_type(frame_->pts, header.stamp));

    int ret = avcodec_send_frame(codecContext_, frame_);
    if (measurePerformance_) {
      t3 = clock.now();
      tdiffSendFrame_.update((t3 - t2).seconds());
    }
    // now drain all packets
    while (ret == 0) {
      ret = drainPacket(header, width, height);
    }
    if (measurePerformance_) {
      const rclcpp::Time t4 = clock.now();
      tdiffTotal_.update((t4-t0).seconds());
    }
  }

  int FFMPEGEncoder::drainPacket(const std_msgs::msg::Header &header,
                                 int width, int height) {
      rclcpp::Clock clock(RCL_SYSTEM_TIME);
    rclcpp::Time t0, t1, t2;
    if (measurePerformance_) {
      t0 = clock.now();
    }
    int ret = avcodec_receive_packet(codecContext_, &packet_);
    if (measurePerformance_) {
      t1 = clock.now();
      tdiffReceivePacket_.update((t1 - t0).seconds());
    }
    const AVPacket &pk = packet_;
    if (ret == 0 && packet_.size > 0) {
      FFMPEGPacket *packet = new FFMPEGPacket;
      FFMPEGPacketConstPtr pptr(packet);
      packet->data.resize(packet_.size);
      packet->img_width =  width; 
      packet->img_height = height;
      packet->pts        = pk.pts;
      packet->flags      = pk.flags;
      memcpy(&(packet->data[0]), packet_.data, packet_.size);
      if (measurePerformance_) {
        t2 = clock.now();
        totalOutBytes_ += packet_.size;
        tdiffCopyOut_.update((t2 - t1).seconds());
      }
      packet->header = header;
      auto it = ptsToStamp_.find(pk.pts);
      if (it != ptsToStamp_.end()) {
        packet->header.stamp = it->second;
        packet->encoding = codecName_ ;
        callback_(pptr);  // deliver packet callback
        if (measurePerformance_) {
          const rclcpp::Time t3 = clock.now();
          tdiffPublish_.update((t3 - t2).seconds());
        }
        ptsToStamp_.erase(it);
      } else {
        RCLCPP_ERROR(logger, "pts %i has no time stamp!", pk.pts);
      }
      av_packet_unref(&packet_); // free packet allocated by encoder
      av_init_packet(&packet_); // prepare next one
    }
    return (ret);
  }

  AVPixelFormat FFMPEGEncoder::pixFormat() const
  {
      return pixFormat_;
  }

  void FFMPEGEncoder::setPixFormat(const AVPixelFormat &pixFormat)
  {
      pixFormat_ = pixFormat;
  }

  void FFMPEGEncoder::printTimers(const std::string &prefix) const {
      Lock lock(mutex_);
      std::stringstream ss;
      ss << prefix
         << " pktsz: " << totalOutBytes_ / frameCnt_
                    << " compr: " << totalInBytes_ / (double)totalOutBytes_
                    << " debay: " << tdiffDebayer_
                    << " fmcp: " << tdiffFrameCopy_
                    << " send: " << tdiffSendFrame_
                    << " recv: " << tdiffReceivePacket_
                    << " cout: " << tdiffCopyOut_
                    << " publ: " << tdiffPublish_
                    << " tot: " << tdiffTotal_
      ;
    RCLCPP_INFO(logger, ss.str());
  }
  void FFMPEGEncoder::resetTimers() {
    Lock lock(mutex_);
    tdiffDebayer_.reset();
    tdiffFrameCopy_.reset();
    tdiffSendFrame_.reset();
    tdiffReceivePacket_.reset();
    tdiffCopyOut_.reset();
    tdiffPublish_.reset();
    tdiffTotal_.reset();
    frameCnt_ = 0;
    totalOutBytes_ = 0;
    totalInBytes_ = 0;
  }
}  // namespace
