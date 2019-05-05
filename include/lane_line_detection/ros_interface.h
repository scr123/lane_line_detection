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
 * @file ros_interface.h
 * @brief Header file for ROS interface.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

/// System
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

/// Project
#include "lane_line_detection/lane_line_detection.h"
#include "lane_line_detection/line_type.h"
#include "lane_line_detection/FilterParamsConfig.h"
#include "lane_line_detection/LaneLine.h"
#include "lane_line_detection/LaneLines.h"

namespace lane_line_detection {
class RosInterface {
 public:
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _p_nh  The p nh
   */
  explicit RosInterface(const ros::NodeHandle& _p_nh);

  /**
   * @brief      Destroys the object.
   */
  ~RosInterface();

 private:
  /**
   * Node Handles
   */
  ros::NodeHandle p_nh_;
  ros::NodeHandle viz_nh_;

  /**
   * Callback Queues
   */
  ros::CallbackQueue viz_queue_;

  /**
   * Timers
   */
  ros::Timer viz_timer_;

  /**
   * Dynamic Reconfigure
   */
  dynamic_reconfigure::Server<FilterParamsConfig> srv_;

  /**
   * Subs & Pubs
   */
  ros::Subscriber camera_info_sub_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  ros::Publisher lane_lines_pub_;

  /**
   * Params
   */
  /// Namespaces & Topics
  std::string node_name_;
  std::string camera_info_sub_topic_;
  std::string image_sub_topic_;
  std::string image_pub_topic_;
  std::string lane_lines_topic_;
  std::string lane_viz_topic_;

  /// Frame IDs
  std::string mount_frame_id_;
  std::string camera_frame_id_;

  /// Timer Rates
  double viz_rate_hz_;

  /// Camera Calibration
  CameraParam camera_info_;
  CalibParam K_, D_, R_, P_;

  /// Filter Weights
  RoiParam roi_;
  BoundParam yellow_bound_;
  BoundParam white_bound_;
  BlurParam blur_;
  CannyParam canny_;
  HoughParam hough_;

  /**
   * @brief      Loads parameters.
   */
  void LoadParams();

  /**
   * @brief      Initializes all class object shared_ptrs
   */
  void InitObjects();

  /**
   * @brief      Initializes all ROS Publishers & Subscribers
   */
  void InitPubSub();

  /**
   * @brief      Starts callbacks.
   */
  void StartCallbacks();

  /**
   * @brief      Populates LineLines msg using LineData vector.
   *
   * @param      _lines     The lines
   * @param[in]  _line_map  The line map
   */
  void PopulateLaneMsg(LaneLines* _lines,
    const std::map<uint8_t, LineData>& _line_map);

  /**
   * @brief      Wrapper for getting the lane lines.
   *
   * @return     The lane lines.
   */
  const std::map<uint8_t, LineData> GetLaneLines();

  /**
   * @brief      Draws lane lines.
   *
   * @param[in]  _line_map  The line map
   */
  void DrawLaneLines(const std::map<uint8_t, LineData>& _line_map);

  /**
   * @brief      Callback for Dynamic Reconfigure params.
   *
   * @param      _config  The configuration
   * @param[in]  _level   The level
   */
  void DynamicReconfigureCB(FilterParamsConfig& _config,
    uint32_t _level);

  /**
   * @brief      Callback for camera information.
   *
   * @param[in]  _msg  The message
   */
  void CameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr& _msg);

  /**
   * @brief      Callback for video stream.
   *
   * @param[in]  _msg  The message
   */
  void VideoCB(const sensor_msgs::Image::ConstPtr& _msg);

  /**
   * @brief      Callback Rviz markers.
   *
   * @param[in]  _event  The event
   */
  void VizCB(const ros::TimerEvent& _event);

  /**
   * @brief      Verifies ROS parameter exists.
   *
   * @param[in]  _ns        namespace string
   * @param[in]  _param     The parameter
   * @param      _variable  The variable
   *
   * @tparam     T          ROS parameter type
   */
  template<typename T>
  void VerifyParam(const std::string& _ns,
    const std::string& _param,
    T &_variable) {
    if (!p_nh_.getParam(_ns + _param, _variable)) {
      ROS_ERROR("[%s]: Cannot retrieve value for param [%s]. Exiting...",
        ros::this_node::getName().c_str(), _param.c_str());
      exit(1);
    }
  }

  /// Shared Pointers
  std::shared_ptr<LaneLineDetection> lld_ptr_;
  std::shared_ptr<ros::AsyncSpinner> as_ptr_;
  std::shared_ptr<rviz_visual_tools::RvizVisualTools> rvt_ptr_;

  /// Mutexes
  std::mutex viz_mutex_;
};
}   // namespace lane_line_detection

#endif  // ROS_INTERFACE_H_