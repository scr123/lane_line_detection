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
 * @file least_squares.h
 * @brief Header file for least_squares.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef LEAST_SQUARES_H_
#define LEAST_SQUARES_H_

#include <Eigen/Dense>

#include <cmath>
#include <vector>

#include <opencv2/core/types.hpp>

namespace lane_line_detection {
class LeastSquares {
 public:
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _order  The order
   */
  explicit LeastSquares(const size_t _order);

  /**
   * @brief      Gets the coefficients.
   *
   * @return     The coefficients.
   */
  const std::vector<float>& GetCoefficients() const;

  /**
   * @brief      Gets the order.
   *
   * @return     The order.
   */
  const size_t& GetOrder() const;

  /**
   * @brief      Sets the order.
   *
   * @param[in]  _order  The order
   */
  void SetOrder(const size_t _order);

  /**
   * @brief      Calculates the coefficients.
   *
   * @param[in]  _data  The data
   */
  void CalcCoefficients(const std::vector<cv::Point2f>& _data);

 private:
  /// Polynomial order k
  size_t k_;

  /// Coefficient vector
  std::vector<float> coefficients_;
};
}   // namespace lane_line_detection

#endif  // LEAST_SQUARES_H_