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
// A land_mark_feature is a caracteristic of an object in the environment/map
// It can be a point cloud, a transform, etc. depending on the application and the sensor used
//

#ifndef LAND_MARK_FEATURE_HPP_
#define LAND_MARK_FEATURE_HPP_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <string>
#include <iostream>
#include <any>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <antikythera_msgs/msg/land_mark_feature.hpp>

namespace antikythera
{

constexpr const char * FEATURE_TYPE_UNKNOWN = "Unknown";
constexpr const char * FEATURE_TYPE_POINT_CLOUD = "PointCloud";
constexpr const char * FEATURE_TYPE_TRANSFORM = "Transform";

// Abstract class definition for a feature of a LandMarkObject
class LandMarkFeature
{
public:
  // CONSTRUCTORS
  LandMarkFeature() = default;

  explicit LandMarkFeature(const std::string & feature_type)
  : feature_type(feature_type) {}

  // DESTRUCTOR
  virtual ~LandMarkFeature() = default;

  // PRINT
  virtual void print() const = 0;  // Print details of the feature

  // DATA ACCESS
  template<typename T>
  std::shared_ptr<T> get_feature_data() const
  {
    return std::static_pointer_cast<T>(get_feature_data_impl());
  }

  template<typename T>
  void set_feature_data(const std::shared_ptr<T> & feature_data)
  {
    set_feature_data_impl(std::static_pointer_cast<void>(feature_data));
  }


  // FEATURE TYPE ACCESS
  std::string get_feature_type() const {return std::string(feature_type);}
  void set_feature_type(const std::string & feature_type) {this->feature_type = feature_type;}

  // ROS CONVERSIONS
  // Get the ROS message corresponding to the feature
  virtual antikythera_msgs::msg::LandMarkFeature toROSMsg() const = 0;

  // Set the feature from a ROS message
  virtual void fromROSMsg(const antikythera_msgs::msg::LandMarkFeature & msg) = 0;

protected:
  std::string feature_type;
  virtual std::shared_ptr<void> get_feature_data_impl() const = 0;
  virtual void set_feature_data_impl(const std::shared_ptr<void> & feature_data) = 0;
};
}  // namespace antikythera

#endif  // LAND_MARK_FEATURE_HPP_
