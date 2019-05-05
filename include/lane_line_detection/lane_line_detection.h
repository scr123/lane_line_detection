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
 * @file lane_line_detection.h
 * @brief Header file for lane_line_detection.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef LANE_LINE_DETECTION_H_
#define LANE_LINE_DETECTION_H_

/// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <utility>
#include <vector>

/// Library
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

/// Project
#include "lane_line_detection/filter_types.h"
#include "lane_line_detection/line_type.h"
#include "lane_line_detection/least_squares.h"

namespace lane_line_detection {
/**
 * @brief      Class for lane line detection.
 */
class LaneLineDetection {
 public:
  const int LEFT = 1;
  const int RIGHT = -1;

  LaneLineDetection(const RoiParam& _roi_param,
    const BoundParam& _yellow_param,
    const BoundParam& _white_param,
    const BlurParam& _gaussian_blur_param,
    const CannyParam& _canny_edge_param,
    const HoughParam& _hough_lines_param);


  /**
   * @brief      Processes image.
   *
   * @param[in]  _mat  The matrix
   */
  void ProcessImage(const cv::Mat& _mat);

  /**
   * @brief      Sets the distortion parameters.
   *
   * @param[in]  _cam_meta  The camera meta-data
   * @param[in]  _cam       The camera calibration matrix
   * @param[in]  _dist      The distortion matrix
   * @param[in]  _R         The rectification matrix
   * @param[in]  _P         The projection matrix
   */
  void SetDistParams(const CameraParam& _cam_meta,
    const CalibParam& _cam,
    const CalibParam& _dist,
    const CalibParam& _R,
    const CalibParam& _P);

  /**
   * @brief      Column Pseudo-Map Point Comparision Struct
   */
  struct ColCmpPm {
      inline bool operator() (
        const std::pair<cv::Point2f, cv::Vec4i>& pm1,
        const std::pair<cv::Point2f, cv::Vec4i>& pm2) {
          return (pm1.first.x < pm2.first.x);
      }
  };

  /**
   * @brief      Row Line Comparison Struct
   */
  struct RowCmpLn {
      inline bool operator() (const cv::Vec4i& ln1, const cv::Vec4i& ln2) {
          return (ln1[1] < ln2[1]);
      }
  };

  /**
   * @brief      Column Line Comparison Struct
   */
  struct ColCmpLn {
      inline bool operator() (const cv::Vec4i& ln1, const cv::Vec4i& ln2) {
          return (ln1[0] < ln2[0]);
      }
  };

  /**
   * @brief      Row Point Comparison Struct
   */
  struct RowCmpPt {
      inline bool operator() (const cv::Point& pt1, const cv::Point& pt2) {
          return (pt1.y < pt2.y);
      }
  };

  /**
   * @brief      Column Point Comparison Struct
   */
  struct ColCmpPt {
      inline bool operator() (const cv::Point& pt1, const cv::Point& pt2) {
          return (pt1.x < pt2.x);
      }
  };

  /**
   * @brief      Row and Column Point Comparison Struct
   */
  struct RowColCmpPt {
      inline bool operator() (const cv::Point& pt1, const cv::Point& pt2) {
          return (std::sqrt(std::pow(pt1.x, 2.0) + std::pow(pt1.y, 2.0))
            < std::sqrt(std::pow(pt2.x, 2.0) + std::pow(pt2.y, 2.0)));
      }
  };

  /**
   * @brief      Classifies Hough Transform Lines using LineData criteria.
   *
   * @param[in]  _ht_lines   The height lines
   * @param      _line_data  The line data
   */
  void ClassifyLaneLines(const std::vector<cv::Vec4i>& _ht_lines,
    std::vector<LineData> *_line_data);

  /**
   * @brief      Generates lane lines from line segments
   *
   * @param[in]  _ht_lines      The Hough Transform lines
   * @param      _line_data     The line data
   * @param[in]  _tf            Transformation matrix
   * @param[in]  _index_method  The index method
   */
  void GenerateLaneLines(const std::vector<cv::Vec4i>& _ht_lines,
    std::vector<LineData> *_line_data,
    const cv::Mat& _tf,
    int _index_method);


  /**
   * @brief      Calculates the line handle.
   *
   * @param[in]  _id       The identifier
   * @param[in]  _coefffs  The coefficients
   *
   * @return     The line handle.
   */
  uint8_t CalcLineHandle(int8_t _id, const std::vector<float>& _coeffs);

  /**
   * @brief      Update pixel to meter ratio.
   */
  void UpdatePixelToMeter();

  /**
   * @brief      Populate LineData map.
   *
   * @param[in]  _lines  The lines
   * @param      _map    The map
   */
  void PopulateLaneLines(const std::vector<LineData>& _lines,
    std::map<uint8_t, LineData> *_map);

  /**
   * @brief      Gets the Mat.
   *
   * @return     The Mat.
   */
  const cv::Mat& GetMat() const;

  /**
   * @brief      Gets the lane lines.
   *
   * @return     The lane lines.
   */
  const std::map<uint8_t, LineData>& GetLaneLines() const;

  /**
   * @brief      Sets the ROI.
   *
   * @param[in]  _param  The parameter
   */
  void SetROI(const RoiParam& _param);

  /**
   * @brief      Sets the yellow bound.
   *
   * @param[in]  _param  The parameter
   */
  void SetYellowBound(const BoundParam& _param);

  /**
   * @brief      Sets the white bound.
   *
   * @param[in]  _param  The parameter
   */
  void SetWhiteBound(const BoundParam& _param);

  /**
   * @brief      Sets the gaussian blur.
   *
   * @param[in]  _param  The parameter
   */
  void SetGaussianBlur(const BlurParam& _param);

  /**
   * @brief      Sets the canny.
   *
   * @param[in]  _param  The parameter
   */
  void SetCanny(const CannyParam& _param);

  /**
   * @brief      Sets the hough lines.
   *
   * @param[in]  _param  The parameter
   */
  void SetHoughLines(const HoughParam& _param);

 private:
  /// Processed Image
  cv::Mat mat_;

  /// Camera Info
  CameraParam cam_info_;

  /// Calibration Matrices
  cv::Mat K_, D_, R_, P_;

  /// Pixel to Meter Mapping using Hwy Lane Standards (3.7m seperation)
  double pixel_to_meter_ratio_;
  bool ratio_found_;

  /// Filter Weights
  RoiParam roi_param_;
  BoundParam yellow_param_;
  BoundParam white_param_;
  BlurParam gaussian_blur_param_;
  CannyParam canny_edge_param_;
  HoughParam hough_lines_param_;

  /// Previous Lane Lines
  std::map<int8_t, LineData> prev_lines_;

  /// Least Squares
  std::shared_ptr<LeastSquares> ls_ptr_;

  /// Extracted Lane Lines
  std::map<uint8_t, LineData> lane_lines_;
};
}   // namespace lane_line_detection

#endif  // LANE_LINE_DETECTION_H_