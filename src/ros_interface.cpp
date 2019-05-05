/**
 * MIT License
 *
 * Copyright (c) 2019 Sean Crutchlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ros_interface.cpp
 * @brief Source file for ROS interface.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "lane_line_detection/ros_interface.h"

namespace lane_line_detection {
  RosInterface::RosInterface(const ros::NodeHandle& _p_nh) {
    /// Node Handle Assignment
    p_nh_ = _p_nh;

    /// Callback Queue Assignment
    viz_nh_.setCallbackQueue(&viz_queue_);

    /// Params
    LoadParams();

    /// Initialize Class Objects
    InitObjects();

    /// Initialize Publishers & Subscribers
    InitPubSub();

    /// Starts Callbacks
    StartCallbacks();
}

  RosInterface::~RosInterface() {
    /// Shutdown Subs & Pubs
    camera_info_sub_.shutdown();
    image_sub_.shutdown();
    image_pub_.shutdown();
    lane_lines_pub_.shutdown();

    /// Stop Timers
    viz_timer_.stop();

    /// Stop Asyncspinner
    as_ptr_->stop();
  }

  void RosInterface::LoadParams() {
    /// Namespaces
    p_nh_.param<std::string>("node_name", node_name_,
      "lane_line_detection_node");
    std::string node_ns_ = "/" + node_name_ + "/";

    /// Check Topics
    VerifyParam(node_ns_, "camera_info_sub_topic", camera_info_sub_topic_);
    VerifyParam(node_ns_, "image_sub_topic", image_sub_topic_);
    VerifyParam(node_ns_, "image_pub_topic", image_pub_topic_);
    VerifyParam(node_ns_, "lane_lines_topic", lane_lines_topic_);
    VerifyParam(node_ns_, "lane_viz_topic", lane_viz_topic_);

    /// Check Frame IDs
    VerifyParam(node_ns_, "mount_frame_id", mount_frame_id_);
    VerifyParam(node_ns_, "camera_frame_id", camera_frame_id_);

    /// Check Timer Rates
    VerifyParam(node_ns_, "viz_rate_hz", viz_rate_hz_);

    /// Check Camera Calibration
    VerifyParam(node_ns_, "image_width", camera_info_.width);
    VerifyParam(node_ns_, "image_height", camera_info_.height);
    VerifyParam(node_ns_, "camera_name", camera_info_.name);
    VerifyParam(node_ns_, "distortion_model", camera_info_.dist_model);
    VerifyParam(node_ns_, "pixel_to_meter_ratio",
      camera_info_.pixel_to_meter_ratio);
    VerifyParam(node_ns_, "use_dynamic_ratio",
      camera_info_.use_dynamic_ratio);
    VerifyParam(node_ns_, "is_undistorted", camera_info_.is_undistorted);

    VerifyParam(node_ns_, "camera_matrix/rows", K_.rows);
    VerifyParam(node_ns_, "camera_matrix/cols", K_.cols);
    VerifyParam(node_ns_, "camera_matrix/data", K_.data);

    VerifyParam(node_ns_, "distortion_coefficients/rows", D_.rows);
    VerifyParam(node_ns_, "distortion_coefficients/cols", D_.cols);
    VerifyParam(node_ns_, "distortion_coefficients/data", D_.data);

    VerifyParam(node_ns_, "rectification_matrix/rows", R_.rows);
    VerifyParam(node_ns_, "rectification_matrix/cols", R_.cols);
    VerifyParam(node_ns_, "rectification_matrix/data", R_.data);

    VerifyParam(node_ns_, "projection_matrix/rows", P_.rows);
    VerifyParam(node_ns_, "projection_matrix/cols", P_.cols);
    VerifyParam(node_ns_, "projection_matrix/data", P_.data);

    /// Check Region of Interest (ROI)
    VerifyParam(node_ns_, "roi_x0", roi_.x0);
    VerifyParam(node_ns_, "roi_y0", roi_.y0);
    VerifyParam(node_ns_, "roi_x1", roi_.x1);
    VerifyParam(node_ns_, "roi_y1", roi_.y1);

    /// Check Yellow Bounds
    VerifyParam(node_ns_, "yellow_lower_r", yellow_bound_.lower_r);
    VerifyParam(node_ns_, "yellow_lower_g", yellow_bound_.lower_g);
    VerifyParam(node_ns_, "yellow_lower_b", yellow_bound_.lower_b);
    VerifyParam(node_ns_, "yellow_upper_r", yellow_bound_.upper_r);
    VerifyParam(node_ns_, "yellow_upper_g", yellow_bound_.upper_g);
    VerifyParam(node_ns_, "yellow_upper_b", yellow_bound_.upper_b);

    /// Check White Bounds
    VerifyParam(node_ns_, "white_lower_r", white_bound_.lower_r);
    VerifyParam(node_ns_, "white_lower_g", white_bound_.lower_g);
    VerifyParam(node_ns_, "white_lower_b", white_bound_.lower_b);
    VerifyParam(node_ns_, "white_upper_r", white_bound_.upper_r);
    VerifyParam(node_ns_, "white_upper_g", white_bound_.upper_g);
    VerifyParam(node_ns_, "white_upper_b", white_bound_.upper_b);

    /// Check Gaussian Blur Params
    VerifyParam(node_ns_, "size_m", blur_.size_m);
    VerifyParam(node_ns_, "size_n", blur_.size_n);
    VerifyParam(node_ns_, "sigma_x", blur_.sigma_x);
    VerifyParam(node_ns_, "sigma_y", blur_.sigma_y);

    /// Check Canny Edge Detection Params
    VerifyParam(node_ns_, "thr_1", canny_.thr_1);
    VerifyParam(node_ns_, "thr_2", canny_.thr_2);
    VerifyParam(node_ns_, "aperture_size", canny_.aperture_size);
    VerifyParam(node_ns_, "l2_gradient", canny_.l2_gradient);

    /// Check Hough Line Transform Params
    VerifyParam(node_ns_, "rho", hough_.rho);
    VerifyParam(node_ns_, "theta", hough_.theta);
    VerifyParam(node_ns_, "threshold", hough_.threshold);
    VerifyParam(node_ns_, "min_line_length", hough_.min_line_length);
    VerifyParam(node_ns_, "max_line_gap", hough_.max_line_gap);
  }

  void RosInterface::InitObjects() {
    /// Lane Line Detection
    lld_ptr_ = std::make_shared<LaneLineDetection>(roi_,
      yellow_bound_,
      white_bound_,
      blur_,
      canny_,
      hough_);

    /// Async Spinner
    as_ptr_ = std::make_shared<ros::AsyncSpinner>(
      0, &viz_queue_);  /// Use as many processor cores available

    /// Rviz Visual Tools
    rvt_ptr_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      camera_frame_id_,
      lane_viz_topic_,
      p_nh_);

    /// Setup Rviz Visual Tools
    rvt_ptr_->loadMarkerPub();
    rvt_ptr_->deleteAllMarkers();
    rvt_ptr_->enableBatchPublishing();

    /// Camera Calibration
    lld_ptr_->SetDistParams(camera_info_,
      K_,
      D_,
      R_,
      P_);
  }

  void RosInterface::InitPubSub() {
    /// Subscribers
    camera_info_sub_ = p_nh_.subscribe<sensor_msgs::CameraInfo>(
      camera_info_sub_topic_, 1, &RosInterface::CameraInfoCB, this);
    image_sub_ = p_nh_.subscribe<sensor_msgs::Image>(
      image_sub_topic_, 1, &RosInterface::VideoCB, this);

    /// Publishers
    image_pub_ = p_nh_.advertise<sensor_msgs::Image>(
      image_pub_topic_, 1);
    lane_lines_pub_ = p_nh_.advertise<LaneLines>(
      lane_lines_topic_, 1);
  }

  void RosInterface::StartCallbacks() {
    /// Timers
    viz_timer_ = viz_nh_.createTimer(ros::Duration(1.0 / viz_rate_hz_),
      &RosInterface::VizCB, this);

    /// Dynamic Reconfigure
    srv_.setCallback(boost::bind(&RosInterface::DynamicReconfigureCB,
      this, _1, _2));

    /// Start Asyncspinner
    as_ptr_->start();
  }

  void RosInterface::PopulateLaneMsg(LaneLines* _lines,
    const std::map<uint8_t, LineData>& _line_map) {
    _lines->header.stamp = ros::Time::now();
    _lines->header.frame_id = mount_frame_id_;

    /// Extract lane lines from map and store in vector
    for (auto it : _line_map) {
      LaneLine msg;

      msg.handle = it.second.handle;
      msg.stamp.sec = it.second.timestamp;
      msg.id = it.second.id;
      msg.type = it.second.type;
      msg.coefficients = it.second.coefficients;

      _lines->lane_lines.push_back(msg);
    }
  }

  const std::map<uint8_t, LineData> RosInterface::GetLaneLines() {
    std::map<uint8_t, LineData> data;

    /// Mutex for lane line getter while used by LaneLines publisher
    /// and rviz in seperate threads
    viz_mutex_.lock();
    data = lld_ptr_->GetLaneLines();
    viz_mutex_.unlock();

    return data;
  }

  void RosInterface::DrawLaneLines(const std::map<uint8_t,
    LineData>& _line_map) {
    /// Clear existing markers to not have artifacts
    rvt_ptr_->deleteAllMarkers();

    /// Iterate through map publishing lane line data
    for (auto it : _line_map) {
      /// Line text meta-data
      std::stringstream coeffs_ss;
      std::copy(it.second.coefficients.begin(),
        it.second.coefficients.end(),
        std::ostream_iterator<double>(coeffs_ss, " "));

      std::string line_str = "ID: " + std::to_string(it.second.id)
        + "\nHandle: " + std::to_string(it.second.handle)
        + "\nType: " + g_line_type.find(it.second.type)->second
        + "\nCoeffs: [" + coeffs_ss.str() + "]";

      /// Lane line points for publishing a path
      std::vector<geometry_msgs::Point> geo_pts;
      for (auto pt : it.second.points) {
        geometry_msgs::Point geo_pt;

        geo_pt.x = pt.first;
        geo_pt.y = pt.second;
        geo_pt.z = 0.0f;

        geo_pts.push_back(geo_pt);
      }

      if (!geo_pts.empty()) {
        geometry_msgs::Pose text_pose;
        text_pose.position = geo_pts.front();
        text_pose.orientation = geometry_msgs::Quaternion();

        rvt_ptr_->publishText(text_pose, line_str, rviz_visual_tools::RED,
          rviz_visual_tools::XXXXLARGE);
        rvt_ptr_->publishPath(geo_pts, rviz_visual_tools::GREEN,
          rviz_visual_tools::XLARGE, "Path");

        /// Publish all rviz markers at once (batch publishing)
        rvt_ptr_->trigger();
      }
    }
  }

  void RosInterface::DynamicReconfigureCB(FilterParamsConfig& _config,
      uint32_t _level) {
    /// Region of Interest (ROI)
    roi_ = {_config.roi_x0,
      _config.roi_y0,
      _config.roi_x1,
      _config.roi_y1};

    lld_ptr_->SetROI(roi_);

    /// Yellow Bounds
    yellow_bound_ = {_config.yellow_lower_r,
      _config.yellow_lower_g,
      _config.yellow_lower_b,
      _config.yellow_upper_r,
      _config.yellow_upper_g,
      _config.yellow_upper_b};

    lld_ptr_->SetYellowBound(yellow_bound_);

    /// White Bounds
    white_bound_ = {_config.white_lower_r,
      _config.white_lower_g,
      _config.white_lower_b,
      _config.white_upper_r,
      _config.white_upper_g,
      _config.white_upper_b};

    lld_ptr_->SetWhiteBound(white_bound_);

    /// Gaussian Blur
    blur_ = {_config.size_m,
      _config.size_n,
      _config.sigma_x,
      _config.sigma_y,
      /// BORDER_DEFAULT
      4};

    lld_ptr_->SetGaussianBlur(blur_);

    /// Canny Edge Detection
    canny_ =  {_config.thr_1,
      _config.thr_2,
      _config.aperture_size,
      _config.l2_gradient};

    lld_ptr_->SetCanny(canny_);

    /// Probabilistic Hough Line Transform
    hough_ = {_config.rho,
      _config.theta,
      _config.threshold,
      _config.min_line_length,
      _config.max_line_gap};

    lld_ptr_->SetHoughLines(hough_);
  }

  void RosInterface::CameraInfoCB(
    const sensor_msgs::CameraInfo::ConstPtr& _msg) {
    /// Camera Frame Info
    camera_info_.height = _msg->height;
    camera_info_.width = _msg->width;
    camera_info_.dist_model = _msg->distortion_model;

    /// Camera Intrinsic Properties
    CalibParam K{3, 3, std::vector<float>(_msg->K.begin(), _msg->K.end())};
     K_ = K;

    CalibParam D{1, 5, std::vector<float>(_msg->D.begin(), _msg->D.end())};
    D_ = D;

    CalibParam R{3, 3, std::vector<float>(_msg->R.begin(), _msg->R.end())};
    R_ = R;

    CalibParam P{3, 4, std::vector<float>(_msg->P.begin(), _msg->P.end())};
    P_ = P;

    /**
     * Not Supported:
     *  _msg->binning_x;
     *  _msg->binning_y;
     */

    /// Region of Interest (ROI)
    if (_msg->roi.do_rectify) {
      roi_.x0 = _msg->roi.x_offset;
      roi_.y0 = _msg->roi.y_offset;
      roi_.x1 = _msg->roi.width;
      roi_.y1 = _msg->roi.height;
    }

    /// Set Camera Calibration
    lld_ptr_->SetDistParams(camera_info_,
      K_,
      D_,
      R_,
      P_);

    /// Set ROI
    lld_ptr_->SetROI(roi_);
  }

  void RosInterface::VideoCB(const sensor_msgs::Image::ConstPtr& _msg) {
    /// Pointer for reading image
    cv_bridge::CvImageConstPtr cv_const_ptr;
    /// Pointer for publishing processed image
    cv_bridge::CvImage cv_img;

    /// Msg for storing lane line information
    LaneLines ll_msg;

    /// Convert sensor_msgs::Image to OpenCV image format
    try {
      if (sensor_msgs::image_encodings::isColor(_msg->encoding))
        cv_const_ptr = cv_bridge::toCvShare(_msg,
          sensor_msgs::image_encodings::BGR8);
      else
        cv_const_ptr = cv_bridge::toCvShare(_msg,
          sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());

      return;
    }

    /// Process OpenCV image
    // double start_time = ros::Time::now().toSec();
    lld_ptr_->ProcessImage(cv_const_ptr->image);
    // ROS_INFO("[%s] Time to Process Image [%1.6f]",
    //   ros::this_node::getName().c_str(),
    //   (ros::Time::now().toSec() - start_time));

    /// Populate LaneLines msg
    PopulateLaneMsg(&ll_msg, GetLaneLines());

    /// Publish lanes lines
    lane_lines_pub_.publish(ll_msg);

    /// Publish converted OpenCV image in sensor_msgs::Image format
    cv_img.image = lld_ptr_->GetMat();
    cv_img.encoding = "bgr8";
    cv_img.header.frame_id = camera_frame_id_;
    cv_img.header.stamp = ros::Time::now();
    image_pub_.publish(cv_img.toImageMsg());
  }

  void RosInterface::VizCB(const ros::TimerEvent& _event) {
    /// Draw Lane Lines in Rviz
    DrawLaneLines(GetLaneLines());
  }
}   // namespace lane_line_detection
