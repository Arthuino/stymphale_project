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
// @file point_cloud_feature.cpp
// @brief This file contains the implemention of the PointCloudFeature class
//


#include "point_cloud_feature.hpp"
#include <iostream>

namespace antikythera
{
// PRINT
void PointCloudFeature::print() const
{
  std::cout << "PointCloudFeature with " << cloud->size() << " points." << std::endl;
}

// GETTERS
std::shared_ptr<void> PointCloudFeature::get_feature_data() const
{
  return cloud;
}

std::string PointCloudFeature::get_feature_type() const
{
  return std::string(FEATURE_TYPE_POINT_CLOUD);
}

// SETTERS
void PointCloudFeature::set_feature(const std::shared_ptr<std::any> & feature_data)
{
  try
  {
    cloud = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>(feature_data);
  }
  catch (const std::bad_any_cast& e)
  {
    std::cerr << "Error: feature_data is not a valid PointCloud pointer!" << std::endl;
  }
}

}  // namespace antikythera
