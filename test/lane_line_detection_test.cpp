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
 * @file lane_line_detection_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "lane_line_detection/lane_line_detection.h"

namespace lane_line_detection {
class LaneLineDetectionTest : public ::testing::Test {
 protected:
  LaneLineDetection *lld_ptr;

  virtual void SetUp() {
    lld_ptr = new LaneLineDetection({0, 0, 640, 480},
      {0, 230, 230, 0, 255, 255},
      {23, 88, 0, 65, 146, 204},
      {33, 33, 1, 0, 4},
      {50, 150, 3, false},
      {2.0, 0.0174, 1, 15, 5});
  }

  virtual void TearDown() {
    delete lld_ptr;
  }

  void RunDetection(const cv::Mat& _img) {
    /// See if image can be opened
    if (!_img.data) {
      std::cout << "Could not read file" << std::endl;
      return;
    /// Check if image is empty
    } else if (_img.empty()) {
      std::cout << "Empty Image" << std::endl;
      return;
    /// Process image to find lane lines
    } else {
      lld_ptr->ProcessImage(_img);
    }
    /// Show processed image
    cv::imshow("img", lld_ptr->GetMat());
    cv::waitKey(0);

    /// Store local copy of all lane lines
    std::map<uint8_t, LineData> lines = lld_ptr->GetLaneLines();

    /// Print all lane line information
    std::cout << "LaneLines:";
    for (auto it : lines) {
        std::cout << "\n\thandle = " << std::to_string(it.second.handle)
        << "\n\ttimestamp = " << it.second.timestamp
        << "\n\tid = " << std::to_string(it.second.id)
        << "\n\ttype = " << g_line_type.find(it.second.type)->second
        << "\n\tcoefficents = [ ";

        for (auto itC : it.second.coefficients)
          std::cout << itC << " ";
        std::cout << "]" << std::endl;
    }
  }
};

TEST_F(LaneLineDetectionTest, Straight) {
  cv::Mat input_image = cv::imread("straight_road.png");

  lld_ptr->SetROI({0, 0, 1280, 1280});  /// Set ROI to be size of image
  RunDetection(input_image);

  EXPECT_EQ(lld_ptr->GetLaneLines().size(), 5);
}

/// TODO(Sean): Add more tests where all extrinsic properties
/// are known. Exact positions of lane lines w.r.t. camera.

}   //  namespace lane_line_detection

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
