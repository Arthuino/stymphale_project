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
// @file land_mark_feature.cpp
// @brief This file contains the implementation of the LandMarkFeature class
//

#include "land_mark_feature.hpp"
#include "point_cloud_feature.hpp"
#include <antikythera_msgs/msg/land_mark_feature.hpp>

namespace antikythera
{

// CONSTRUCTORS
LandMarkFeature::LandMarkFeature(const std::string & feature_type)
: feature_type(feature_type) {}

// PRINT
void LandMarkFeature::print() const
{
  if (!feature_type.empty()) {
    std::cout << "Virtual Feature of Type: " << feature_type << std::endl;
  } else {
    std::cout << "Virtual Feature of Type: Unknown" << std::endl;
  }
}


// ROS serialization methods
// Convert a LandMarkFeature object to ROS message format
void LandMarkFeature::toROSMsg(
  const std::shared_ptr<LandMarkFeature> & landMarkFeature,
  antikythera_msgs::msg::LandMarkFeature & msg)
{
  // Set Feature Type
  msg.feature_type = landMarkFeature->get_feature_type();

  // Set Feature Data
  if (msg.feature_type == FEATURE_TYPE_POINT_CLOUD) {
      // dynamic cast to PointCloudFeature
      msg.is_point_cloud = true;
      auto pointCloudFeature = std::dynamic_pointer_cast<PointCloudFeature>(landMarkFeature);
      if (pointCloudFeature) {
          msg.feature_type = FEATURE_TYPE_POINT_CLOUD;
          auto cloud = std::static_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(
                                                          pointCloudFeature->get_feature_data());
          pcl::toROSMsg(* cloud, msg.point_cloud);  // Convert pcl data to ROS msg
      }
  } else if (msg.feature_type == FEATURE_TYPE_TRANSFORM) {
      msg.is_transform = true;
      msg.feature_type = FEATURE_TYPE_TRANSFORM;
      printf("Transform feature not implemented yet\n");
  } else {
      msg.feature_type = FEATURE_TYPE_UNKNOWN;
  }
}

// Convert a ROS message to LandMarkFeature object
void LandMarkFeature::fromROSMsg(
  const antikythera_msgs::msg::LandMarkFeature & msg,
  std::shared_ptr<LandMarkFeature> & landMarkFeature)
{
  if (msg.is_point_cloud) {  // PointCloud Feature
      // Convert feature object to PointCloudFeature
      landMarkFeature = std::make_shared<PointCloudFeature>();

      // Convert data to pcl
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromROSMsg(msg.point_cloud, * cloud);

      // Wrap the shared pointer in std::any and pass it to set_feature
      auto feature_data = std::make_shared<std::any>(cloud);
      std::dynamic_pointer_cast<PointCloudFeature>(landMarkFeature)->set_feature(feature_data);
  } else if (msg.is_transform) {  // Transform Feature
      printf("Transform feature not implemented yet\n");
  }
}


}  // namespace antikythera
