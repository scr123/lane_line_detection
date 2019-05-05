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
 * @file least_squares.cpp
 * @brief Source file for least_squares.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "lane_line_detection/least_squares.h"

namespace lane_line_detection {

  LeastSquares::LeastSquares(const size_t _order) : k_(_order + 1) {}

  const std::vector<float>& LeastSquares::GetCoefficients() const {
    return this->coefficients_;
  }

  const size_t& LeastSquares::GetOrder() const {
    return this->k_;
  }

  void LeastSquares::SetOrder(const size_t _order) {
    this->k_ = _order + 1;
  }

  void LeastSquares::CalcCoefficients(const std::vector<cv::Point2f>& _data) {
    /**
     * Least Squares to generate 3rd order polynomial from gradient
     * 
     * 3rd Order Polynomial:
     *  y = a_0 + (a_1 * x) + (a_2 * x^2) + ... + (a_k * x^k)
     * Least Squares using normal equations:
     *  A_T * Ax' = A_T * b
     */

    /// Residual Matrix - A
    Eigen::MatrixXf A(_data.size(), this->k_);

    /// Coefficient Vector - b
    Eigen::VectorXf b(_data.size());

    for (int row = 0; row < _data.size(); row++) {
      for (int col = 0; col < this->k_; col++) {
        A(row, col) = std::pow(_data[row].x, col);
      }
      b(row) = _data[row].y;
    }

    /// Least Squares - x'
    Eigen::VectorXf x_prime =
      (A.transpose() * A).ldlt().solve(A.transpose() * b);

    /// Convert Eigen Vector to Standard Vector
    std::vector<float> coefficients(x_prime.data(),
      x_prime.data() + x_prime.rows() * x_prime.cols());

    this->coefficients_ = coefficients;
  }
}   // namespace lane_line_detection
