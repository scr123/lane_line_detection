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
 * @file lane_line_detection.cpp
 * @brief Source file for lane_line_detection.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "lane_line_detection/lane_line_detection.h"

namespace lane_line_detection {
  LaneLineDetection::LaneLineDetection(const RoiParam& _roi_param,
    const BoundParam& _yellow_param,
    const BoundParam& _white_param,
    const BlurParam& _gaussian_blur_param,
    const CannyParam& _canny_edge_param,
    const HoughParam& _hough_lines_param) {

    roi_param_ = _roi_param;

    yellow_param_ = _yellow_param;
    white_param_ = _white_param;

    gaussian_blur_param_ = _gaussian_blur_param;
    canny_edge_param_ = _canny_edge_param;
    hough_lines_param_ = _hough_lines_param;

    /// Flag for pixel to meter ratio being set
    pixel_to_meter_ratio_ = 1;
    ratio_found_ = false;

    /// Construct Least Squares 3rd order polynomial object
    ls_ptr_ = std::make_shared<LeastSquares>(3);
  }

  void LaneLineDetection::ProcessImage(const cv::Mat& _mat) {
    /// Copy original image and un-distort if needed
    cv::Mat orig;
    if (!cam_info_.is_undistorted)
      cv::undistort(_mat, orig, this->K_, this->D_);
    else
      orig = _mat;

    /// Select ROI (region of interest) containing primarily lane lines
    orig(cv::Rect(
      cv::Point(std::min(this->roi_param_.x0, orig.cols),
                std::min(this->roi_param_.y0, orig.rows)),
      cv::Point(std::min(this->roi_param_.x1, orig.cols),
                std::min(this->roi_param_.y1, orig.rows)))).copyTo(
    orig);

    /// Second copy of orig that image processing will be performed on
    this->mat_ = orig;

    /// Create Mat variables to store yellow & white only images
    cv::Mat yellow_mask, white_mask;

    /// Convert to HSL
    cv::cvtColor(orig, this->mat_, cv::COLOR_BGR2HLS);

    /// Isolate yellow & white lane markers from HSL image using color masks
    cv::Scalar yellow_lower_bound = cv::Scalar
      (yellow_param_.lower_r,
       yellow_param_.lower_g,
       yellow_param_.lower_b);

    cv::Scalar yellow_upper_bound = cv::Scalar
      (yellow_param_.upper_r,
       yellow_param_.upper_g,
       yellow_param_.upper_b);

    /// Apply upper and lower bounds to yellow color mask
    cv::inRange(this->mat_,
      yellow_lower_bound, yellow_upper_bound,
      yellow_mask);

    cv::Scalar white_lower_bound = cv::Scalar
      (white_param_.lower_r,
       white_param_.lower_g,
       white_param_.lower_b);

    cv::Scalar white_upper_bound = cv::Scalar
      (white_param_.upper_r,
       white_param_.upper_g,
       white_param_.upper_b);

    /// Apply upper and lower bounds to white color mask
    cv::inRange(this->mat_,
      white_lower_bound, white_upper_bound,
      white_mask);

    /// Construct masks for bitwise operations
    cv::Mat yellow_out, white_out, yellow_OR_white, mask_AND_orig;

    /// Create yellow & white masks
    this->mat_.copyTo(yellow_out, yellow_mask);
    this->mat_.copyTo(white_out, white_mask);

    /// Combine yellow & white masks
    cv::bitwise_or(yellow_out, white_out, yellow_OR_white);

    /// Combine isolated HSL image with original
    cv::bitwise_and(orig, yellow_OR_white, mask_AND_orig);
    this->mat_ = mask_AND_orig;

    /// Convert image to grayscale in two steps
    cv::cvtColor(this->mat_, this->mat_, cv::COLOR_HLS2BGR);
    cv::cvtColor(this->mat_, this->mat_, cv::COLOR_BGR2GRAY);

    /// Apply Gaussian Blur to smooth edges
    cv::GaussianBlur(this->mat_, this->mat_,
      cv::Size(gaussian_blur_param_.size_m, gaussian_blur_param_.size_n),
      gaussian_blur_param_.sigma_x, gaussian_blur_param_.sigma_x);

    /// Apply Canny Edge Detection
    cv::Canny(this->mat_, this->mat_,
      canny_edge_param_.thr_1, canny_edge_param_.thr_2);

    /// Store lines found in Probabilistic Hough Line Transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(this->mat_, lines,
      hough_lines_param_.rho,
      hough_lines_param_.theta,
      hough_lines_param_.threshold,
      hough_lines_param_.min_line_length,
      hough_lines_param_.max_line_gap);

    /// Convert back to BGR for visualization purposes
    cv::cvtColor(this->mat_, this->mat_, cv::COLOR_GRAY2BGR);

    /// Classify lane lines & populate respective data type
    std::vector<LineData> extracted_lane_lines;
    ClassifyLaneLines(lines, &extracted_lane_lines);
    PopulateLaneLines(extracted_lane_lines, &(this->lane_lines_));

    /// Convert vector of lane lines to map using lane ID as key
    std::map<int8_t, LineData> tmp_map;
    for (auto it : extracted_lane_lines) {
      tmp_map.insert(std::make_pair(it.id, it));
    }

    /// Store map of most recent lane lines as previous timestep
    this->prev_lines_ = tmp_map;

    /// Update pixel to meter ratio based off of lane line seperation
    if (cam_info_.use_dynamic_ratio)
      UpdatePixelToMeter();
  }

  void LaneLineDetection::SetDistParams(const CameraParam& _cam_meta,
    const CalibParam& _cam,
    const CalibParam& _dist,
    const CalibParam& _R,
    const CalibParam& _P) {

    this->cam_info_ = _cam_meta;

    this->K_.create(_cam.rows, _cam.cols, CV_32F);
    std::memcpy(this->K_.data, &_cam.data,
      (_cam.rows * _cam.cols * sizeof(float)));

    this->D_.create(_dist.rows, _dist.cols, CV_32F);
    std::memcpy(this->D_.data, &_dist.data,
      (_dist.rows * _dist.cols * sizeof(float)));

    this->R_.create(_R.rows, _R.cols, CV_32F);
    std::memcpy(this->R_.data, &_R.data,
      (_R.rows * _R.cols * sizeof(float)));

    this->P_.create(_P.rows, _P.cols, CV_32F);
    std::memcpy(this->P_.data, &_P.data,
      (_P.rows * _P.cols * sizeof(float)));
  }

  void LaneLineDetection::ClassifyLaneLines(
    const std::vector<cv::Vec4i>& _ht_lines,
    std::vector<LineData> *_line_data) {
    /**
     * Categorize Hough Lines as left & right using line slope (point-horizon)
     */
    std::vector<cv::Vec4i> left_hlt;
    std::vector<cv::Vec4i> right_hlt;

    for (size_t i = 0; i < _ht_lines.size(); i++) {
      double slope = 0.0;
      double dx = _ht_lines[i][2] - _ht_lines[i][0];

      /// Check that denominator is not zero
      if (std::fabs(dx) > 1e-8) {
        slope = ((_ht_lines[i][3] - _ht_lines[i][1]) / dx);
      }

      /// Segregate lane lines based on slope
      if (slope <= 0.0)
        left_hlt.push_back(_ht_lines[i]);
      else
        right_hlt.push_back(_ht_lines[i]);

      /// Draw shapes on image for visualization purposes
      cv::line(this->mat_,
        cv::Point(_ht_lines[i][0], _ht_lines[i][1]),
        cv::Point(_ht_lines[i][2], _ht_lines[i][3]),
        cv::Scalar(0, 0, 255), 1, 8);

      cv::circle(this->mat_,
        cv::Point(_ht_lines[i][2], _ht_lines[i][3]),
        1, cv::Scalar(0, 255, 0), 2, 8);

      cv::circle(this->mat_,
        cv::Point(_ht_lines[i][0], _ht_lines[i][1]),
        1, cv::Scalar(0, 255, 0), 2, 8);
    }

    /**
     * Assign four source vertices to be used in perspective transform on
     * points (lane lines) checking if left and right vectors are populated.
     * 
     * Convention:
     *      [0]       [1]
     *      
     *      [3]       [2]
     * 
     */
    std::vector<cv::Point2f> src_vertices(4);
    if (left_hlt.empty() && right_hlt.empty()) {
      return;
    } else if (left_hlt.empty()) {
      std::sort(right_hlt.begin(), right_hlt.end(), ColCmpLn());

      src_vertices[2] = cv::Point2f(right_hlt.back().operator[](2),
          right_hlt.back().operator[](3));
      /// Mirror source vertex from bottom right due to bottom left being empty
      src_vertices[3] = cv::Point2f((this->mat_.cols - src_vertices[2].x),
          src_vertices[2].y);

      std::sort(right_hlt.begin(), right_hlt.end(), RowCmpLn());

      src_vertices[1] = cv::Point2f(right_hlt.front().operator[](0),
        right_hlt.front().operator[](1));
      /// Mirror source vertex from top right due to top left being empty
      src_vertices[0] = cv::Point2f((this->mat_.cols - src_vertices[1].x),
        src_vertices[1].y);
    } else if (right_hlt.empty()) {
      std::sort(left_hlt.begin(), left_hlt.end(), ColCmpLn());

      src_vertices[3] = cv::Point2f(left_hlt.front().operator[](0),
          left_hlt.front().operator[](1));
      /// Mirror source vertex from bottom left due to bottom right being empty
      src_vertices[2] = cv::Point2f((this->mat_.cols - src_vertices[3].x),
          src_vertices[3].y);

      std::sort(left_hlt.begin(), left_hlt.end(), RowCmpLn());

      src_vertices[0] = cv::Point2f(left_hlt.front().operator[](0),
        left_hlt.front().operator[](1));
      /// Mirror source vertex from top left due to top right being empty
      src_vertices[1] = cv::Point2f((this->mat_.cols - src_vertices[0].x),
        src_vertices[0].y);
    } else {
      /**
       * Sort Hough Lines in column order to select frame's
       * bottom vertices for image warping (2D projection)
       */
      std::sort(left_hlt.begin(), left_hlt.end(), ColCmpLn());
      std::sort(right_hlt.begin(), right_hlt.end(), ColCmpLn());

      src_vertices[2] = cv::Point2f(right_hlt.back().operator[](2),
          right_hlt.back().operator[](3));

      src_vertices[3] = cv::Point2f(left_hlt.front().operator[](0),
          left_hlt.front().operator[](1));
       /**
        * Sort Hough Lines in row order to select frame's
        * top vertices for image warping (2D projection)
        */
      std::sort(left_hlt.begin(), left_hlt.end(), RowCmpLn());
      std::sort(right_hlt.begin(), right_hlt.end(), RowCmpLn());

      src_vertices[0] = cv::Point2f(left_hlt.front().operator[](0),
        left_hlt.front().operator[](1));
      src_vertices[1] = cv::Point2f(right_hlt.front().operator[](0),
        right_hlt.front().operator[](1));
    }
    /// Draw vertices for visualization purposes
    for (auto it : src_vertices)
      cv::circle(this->mat_, it, 5, cv::Scalar(255, 0, 0), 3, 8);

    /**
     * Assign four destination vertices to be used in perspective transform on
     * points (lane lines).
     */
    std::vector<cv::Point2f> dst_vertices(4);
    dst_vertices[0] = cv::Point2f(0, 0);
    dst_vertices[1] = cv::Point2f(0, this->cam_info_.width);
    dst_vertices[2] = cv::Point2f(this->cam_info_.height,
      this->cam_info_.width);
    dst_vertices[3] = cv::Point2f(this->cam_info_.height, 0);

    /// Define destination Mat the size of the input image
    cv::Mat dst(this->cam_info_.height, this->cam_info_.width, CV_8UC3);

    /// Generate transformation matrix
    cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);

    /// Generate lane lines from left and right Hough Lines
    if (!left_hlt.empty())
      GenerateLaneLines(left_hlt, _line_data, M, LEFT);
    if (!right_hlt.empty())
      GenerateLaneLines(right_hlt, _line_data, M, RIGHT);

    // cv::warpPerspective(this->mat_, dst, M, dst.size(),
    //  cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    // this->mat_ = dst;
  }

  void LaneLineDetection::GenerateLaneLines(
    const std::vector<cv::Vec4i>& _ht_lines,
    std::vector<LineData> *_line_data,
    const cv::Mat& _tf,
    int _index_method) {
    /// Object capturing all lane line data
    LineData line;
    /// Pair of a Point and Hough Line, where transformation
    /// will be applied to the Point only
    std::vector<std::pair<cv::Point2f, cv::Vec4i>> points_and_ht;

    /// Populate Point and Hough Line pairs
    for (auto it : _ht_lines) {
      points_and_ht.push_back(std::make_pair(
        cv::Point2f(it.operator[](0), it.operator[](1)),
        it));
    }

    /// Create vector of points only for perspective transform
    std::vector<cv::Point2f> points_only;
    for (auto it : points_and_ht)
      points_only.push_back(it.first);

    if (!points_only.empty()) {
      /// Data structure for lane lines containing respective points
      std::vector<std::vector<cv::Point2f>> lines;

      /// Transform points (point A in line segment AB)
      cv::perspectiveTransform(points_only, points_only, _tf);

      /// Write transformed points back to first element in pair for sorting
      for (size_t i = 0; i < points_and_ht.size(); i++)
        points_and_ht[i].first = points_only[i];

      /// Sort vector of pairs by first element for lane line segregation
      std::sort(points_and_ht.begin(), points_and_ht.end(), ColCmpPm());

      /// Criteria for segregating points into respective lane lines
      std::vector<cv::Point2f> line_points;
      for (size_t i = 0; i < points_and_ht.size() - 1; i++) {
        if (points_and_ht[i + 1].first.y > points_and_ht[i].first.y) {
          /**
           * Convert camera coordinate system to R.H.R.
           *   1) Switch x & y
           *   2) x-axis
           *       a) 0 --> this->mat_.rows
           *       b) this->mat_.rows --> 0
           *   3) y-axis
           *       a) LEFT = positive y-axis
           *           i) 0 --> this->mat_.cols
           *          ii) this->mat_.cols --> 0
           *       b) RIGHT = negative y-axis
           *           i) 0 --> 0
           *          ii) this->mat_.cols --> -1 * (this->mat_.cols)
           */
          float sign = (_index_method == LEFT ? 1.0f : -1.0f);
          float x_0 = this->mat_.rows;
          float y_0 = (this->mat_.cols / 2.0);

          line_points.push_back(cv::Point2f(
            (x_0 - points_and_ht[i].second.operator[](1)),
            (sign * std::fabs(points_and_ht[i].second.operator[](0) - y_0))));

          line_points.push_back(cv::Point2f(
            (x_0 - points_and_ht[i].second.operator[](3)),
            (sign * std::fabs(points_and_ht[i].second.operator[](2) - y_0))));

        } else {
          /// If conditional is not met, make sure empty line is not
          /// pushed back
          if (!line_points.empty()) {
            lines.push_back(line_points);
            line_points.resize(0);
          }
        }
      }

      /// Populate data for all lane lines
      for (size_t i = 0; i < lines.size(); i++) {
        /// Time (seconds only)
        line.timestamp = (std::chrono::duration<double>((
          std::chrono::system_clock::now()).time_since_epoch())).count();

        /// ID
        if (_index_method == LEFT)
          line.id = lines.size() - i;  /// ascending order to left
        else
          line.id = -(i + 1);          /// descending order to right

        /// TODO(Sean): Use Deep Learning for classification of lane
        /// line types.
        /// Type
        line.type = WHITE_SOLID;

        /// TODO(Sean): transform these points once depth information
        /// is present.
        /// Transform line points to use the 2D projection frame
        // cv::perspectiveTransform(lines[i], lines[i], _tf);

        /// Really a ColCmp, where points are converted to R.H.R prior
        std::sort(lines[i].begin(), lines[i].end(), RowCmpPt());

        /**
         * Use pixel to meter constant for scaling.
         * 
         * TODO(Sean): Provide pixel to meter mapping
         *  that will be correct for information other
         *  than polynomial c0. Constant does not map
         *  linearly when approaching point horizon of
         *  lane line, or in the 2D projection. Really
         *  need a stereo camera.
         */
        /// Points
        for (auto it : lines[i]) {
          if (cam_info_.use_dynamic_ratio) {
            it.x /= pixel_to_meter_ratio_;
            it.y /= pixel_to_meter_ratio_;
          } else {
            it.x /= cam_info_.pixel_to_meter_ratio;
            it.y /= cam_info_.pixel_to_meter_ratio;
          }
          line.points.push_back(std::make_pair(it.x, it.y));
        }
        /// Calculate Polynomial Coefficients
        ls_ptr_->CalcCoefficients(lines[i]);
        line.coefficients = ls_ptr_->GetCoefficients();

        /// Assign Handle
        line.handle = CalcLineHandle(line.id, line.coefficients);

        _line_data->push_back(line);
      }
    }
  }

  uint8_t LaneLineDetection::CalcLineHandle(int8_t _id,
    const std::vector<float>& _coeffs) {
    /// Find lane line with same ID from previous frame
    auto ret = this->prev_lines_.find(_id);

    /// If the ID is found, use thresholding to see how much lane line c0 moved
    if (ret != this->prev_lines_.end()
      && ((std::fabs(ret->second.coefficients.at(0) - _coeffs.at(0))
        / cam_info_.pixel_to_meter_ratio) < 0.1)) {
        return ret->second.handle;
    } else {
      /// If the lane line c0 moved beyond threshold, re-assign handle
      uint8_t max_handle = 0;
      for (auto it : this->prev_lines_)
        max_handle = std::max(max_handle, it.second.handle);

      return ++max_handle;
    }
  }

  void LaneLineDetection::UpdatePixelToMeter() {
    /// Find left & right lanes closest to vehicle
    auto left = this->prev_lines_.find(LEFT);
    auto right = this->prev_lines_.find(RIGHT);

    /// If both lane lines are found
    if ((left != this->prev_lines_.end())
        && (right != this->prev_lines_.end())
        && (left->second.points.size() > 0)
        && (right->second.points.size() > 0)) {
      /**
       * Notice points.front() is used while the image coordinate
       * system has converted from a camera, to a R.H.R. convention.
       * In the world frame, lane lines begin at the bottom of the image.
       * Distance between the first left & right lanes lines are 3.7 meters
       * for most US roads. Also, the ratio is multiplied by the latest
       * pixel to meter ratio in order to convert back to frame pixels for
       * the new calculation.
       */
      double ratio = (pixel_to_meter_ratio_
        * ((left->second.points.front().second
        + std::fabs(right->second.points.front().second))) / 3.7);

      /// Make sure ratio is not 0 which would cause division issues
      if (std::fabs(ratio) > 1e-8) {
        pixel_to_meter_ratio_ = ratio;
        ratio_found_ = true;
      }
    }
  }

  void LaneLineDetection::PopulateLaneLines(const std::vector<LineData>& _lines,
    std::map<uint8_t, LineData> *_map) {
    for (auto it : _lines) {
      auto ret = _map->insert(std::make_pair(it.id, it));

      /// Overwrite lane line for specified ID
      if (ret.second == false) {
        ret.first->second = it;
      }
    }
  }

  const cv::Mat& LaneLineDetection::GetMat() const {
    return this->mat_;
  }

  const std::map<uint8_t, LineData>& LaneLineDetection::GetLaneLines() const {
    /// If ratio for pixel to meter is not yet found, and dynamic ratio
    /// is being used, return empty lane lines
    if (ratio_found_ || !cam_info_.use_dynamic_ratio) {
      return this->lane_lines_;
    } else {
      static std::map<uint8_t, LineData> empty_map;
      return empty_map;
    }
  }

  void LaneLineDetection::SetROI(const RoiParam& _param) {
    this->roi_param_ = _param;
  }

  void LaneLineDetection::SetYellowBound(const BoundParam& _param) {
    this->yellow_param_ = _param;
  }

  void LaneLineDetection::SetWhiteBound(const BoundParam& _param) {
    this->white_param_ = _param;
  }

  void LaneLineDetection::SetGaussianBlur(const BlurParam& _param) {
    this->gaussian_blur_param_ = _param;
  }

  void LaneLineDetection::SetCanny(const CannyParam& _param) {
    this->canny_edge_param_ = _param;
  }

  void LaneLineDetection::SetHoughLines(const HoughParam& _param) {
    this->hough_lines_param_ = _param;
  }
}   // namespace lane_line_detection
