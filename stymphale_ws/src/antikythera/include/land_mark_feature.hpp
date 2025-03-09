// MIT License
//
// Copyright (c) 2025 Arthur VIGOUROUX
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// @file land_mark_feature.hpp
// @brief This file contains the definition of the abstract LandMarkFeature class
//

#ifndef LAND_MARK_FEATURE_HPP_
#define LAND_MARK_FEATURE_HPP_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <string>
#include <iostream>
#include <any>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <antikythera_msgs/msg/land_mark_feature.hpp>

// TODO(arthuino) : Check for any direct instantiation of Abstract class LandMarkFeature

namespace antikythera
{

constexpr const char* FEATURE_TYPE_UNKNOWN = "Unknown";
constexpr const char* FEATURE_TYPE_POINT_CLOUD = "PointCloud";
constexpr const char* FEATURE_TYPE_TRANSFORM = "Transform";

// Abstract class definition for a feature of a LandMarkObject
class LandMarkFeature
{
public:

  // CONSTRUCTORS
  LandMarkFeature() = default;

  explicit LandMarkFeature(const std::string & feature_type);

  // DESTRUCTOR
  virtual ~LandMarkFeature() = default;

  // PRINT
  virtual void print() const;  // Print details of the feature

  // GETTERS
  [[nodiscard]] virtual std::shared_ptr<void> get_feature_data() const = 0; 
  [[nodiscard]] virtual std::string get_feature_type() const = 0;

  // SETTERS
  virtual void set_feature(const std::shared_ptr<std::any> & feature_data) = 0;  

  // ROS CONVERSIONS
  static void toROSMsg(
    const std::shared_ptr<LandMarkFeature>& landMarkFeature,
    antikythera_msgs::msg::LandMarkFeature& msg);

  static void fromROSMsg(
    const antikythera_msgs::msg::LandMarkFeature& msg,
    std::shared_ptr<LandMarkFeature>& landMarkFeature);


protected:
  std::string feature_type;
};
}  // namespace antikythera

#endif  // LAND_MARK_FEATURE_HPP_
