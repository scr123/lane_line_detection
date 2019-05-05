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
 * @file line_type.h
 * @brief Header file for line_type.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef LINE_TYPE_H_
#define LINE_TYPE_H_

/// System
#include <map>
#include <string>
#include <vector>
#include <utility>

namespace lane_line_detection {

/**
 * Lane Lines Types
 */
const uint8_t WHITE_SOLID = 1;
const uint8_t WHITE_DASHED = 2;
const uint8_t YELLOW_SOLID = 3;
const uint8_t YELLOW_DASHED = 4;

/**
 * Map to lookup lane line types
 */
const std::map <uint8_t, std::string> g_line_type = {
  {WHITE_SOLID,   "WHITE_SOLID"},
  {WHITE_DASHED,   "WHITE_DASHED"},
  {YELLOW_SOLID,   "YELLOW_SOLID"},
  {YELLOW_DASHED,   "YELLOW_DASHED"}
};

/**
 * @brief      Structure to hold lane line meta-data
 */
struct LineData {
  uint8_t handle = 0;
  int8_t id = 0;
  double timestamp = 0.0;
  uint8_t type = 0;
  std::vector<float> coefficients;
  std::vector<std::pair<float, float>> points;
};
}   // namespace lane_line_detection

#endif  // LINE_TYPE_H_