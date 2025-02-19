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
#include <antikythera_msgs/msg/land_mark_feature.hpp>

namespace antikythera
{

LandMarkFeature::~LandMarkFeature() = default;

LandMarkFeature::LandMarkFeature() = default;

LandMarkFeature::LandMarkFeature(const std::string & feature_type, FeatureData feature_data)
: feature_type(feature_type), feature_data(feature_data) {}

void LandMarkFeature::print() const
{
  if (!feature_type.empty()) {
    std::cout << "Feature Type: " << feature_type << std::endl;
  } else {
    std::cout << "Feature Type: Unknown" << std::endl;
  }
}

std::string LandMarkFeature::get_feature_type() const
{
  return feature_type;
}

FeatureData LandMarkFeature::get_feature() const
{
  return feature_data;
}

void LandMarkFeature::set_feature(FeatureData feature)
{
  feature_data = feature;
  // set the feature type based on the variant type
  if (std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>::Ptr>(feature)) {
    feature_type = "PointCloud";
  } else if (std::holds_alternative<geometry_msgs::msg::TransformStamped>(feature)) {
    feature_type = "TransformStamped";
  } else {
    feature_type = "Unknown";
  }
}


// ROS serialization methods
void LandMarkFeature::toROSMsg(
  const LandMarkFeature & landMarkFeature,
  antikythera_msgs::msg::LandMarkFeature & msg)
{
  // Set Feature Type
  msg.feature_type = landMarkFeature.get_feature_type();
  // Set Flags
  msg.is_point_cloud = std::holds_alternative<pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    landMarkFeature.get_feature()
  );

  msg.is_transform = std::holds_alternative<geometry_msgs::msg::TransformStamped>(
    landMarkFeature.get_feature()
  );

  // Set Feature Data
  if (msg.is_point_cloud) {       // PointCloud Feature
    auto cloud = std::get<pcl::PointCloud<pcl::PointXYZ>::Ptr>(landMarkFeature.get_feature());
    pcl::toROSMsg(*cloud, msg.point_cloud);

  } else if (msg.is_transform) {  // Transform Feature
    printf("Transform feature not implemented yet\n");
  }
}

void LandMarkFeature::fromROSMsg(
  const antikythera_msgs::msg::LandMarkFeature & msg,
  LandMarkFeature & landMarkFeature)
{
  if (msg.is_point_cloud) {   // PointCloud Feature
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(msg.point_cloud, *cloud);
    landMarkFeature.set_feature(cloud);

  } else if (msg.is_transform) {  // Transform Feature
    printf("Transform feature not implemented yet\n");
  }
}

}  // namespace antikythera
