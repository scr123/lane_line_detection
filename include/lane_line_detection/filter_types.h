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
 * @file filter_types.h
 * @brief Header file for filter_types.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef FILTER_TYPES_H_
#define FILTER_TYPES_H_

#include <string>
#include <vector>

namespace lane_line_detection {

/**
 * @brief      Structure for camera calibration meta-data
 */
struct CameraParam {
  int width = 640;
  int height = 480;
  std::string name;
  std::string dist_model;
  double pixel_to_meter_ratio = 100;
  bool use_dynamic_ratio = false;
  bool is_undistorted = true;
};

/**
 * @brief      Structure for camera calibration
 */
struct CalibParam {
  int rows;
  int cols;
  std::vector<float> data;
};

/**
 * @brief      Structure for selecting region of interest
 */
struct RoiParam {
  int x0;
  int y0;
  int x1;
  int y1;
};

/**
 * @brief      Structure for upper & lower bounds of color masks
 */
struct BoundParam {
  int lower_r;
  int lower_g;
  int lower_b;
  int upper_r;
  int upper_g;
  int upper_b;
};

/**
 * @brief      Structure for Gaussian Blur filter weights
 */
struct BlurParam {
  int size_m;
  int size_n;
  double sigma_x;
  double sigma_y;
  int border_type;
};

/**
 * @brief      Structure for Canny Edge Detection filter weights
 */
struct CannyParam {
  double thr_1;
  double thr_2;
  int aperture_size;
  bool l2_gradient;
};

/**
 * @brief      Structure for Hough Transform filter weights
 */
struct HoughParam {
  double rho;
  double theta;
  int threshold;
  double min_line_length;
  double max_line_gap;
};
}   // namespace lane_line_detection

#endif  // FILTER_TYPES_H_