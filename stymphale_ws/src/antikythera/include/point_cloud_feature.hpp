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
// @file point_cloud_feature.hpp
// @brief This file contains the definition of the PointCloudFeature class
// This feature is a PCL point cloud. Point cloud can be from a LIDAR, a stereo camera, etc.
//

#ifndef POINT_CLOUD_FEATURE_HPP_
#define POINT_CLOUD_FEATURE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>
#include <memory>

#include "land_mark_feature.hpp"

namespace antikythera
{

class PointCloudFeature : public LandMarkFeature
{
public:
  // CONSTRUCTORS
  PointCloudFeature()
  : LandMarkFeature(FEATURE_TYPE_POINT_CLOUD),
    cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()) {}

  explicit PointCloudFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  : LandMarkFeature(FEATURE_TYPE_POINT_CLOUD), cloud(cloud) {}

  // DESCTRUCTOR
  ~PointCloudFeature() override = default;

  // Correctly declare the print function
  void print() const override;

  // ROS CONVERSIONS
  antikythera_msgs::msg::LandMarkFeature toROSMsg() const override;

  void fromROSMsg(const antikythera_msgs::msg::LandMarkFeature & msg) override;
  std::shared_ptr<void> get_feature_data_impl() const override;
  void set_feature_data_impl(const std::shared_ptr<void> & feature_data) override;

private:
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
};
}  // namespace antikythera

#endif  // POINT_CLOUD_FEATURE_HPP_
