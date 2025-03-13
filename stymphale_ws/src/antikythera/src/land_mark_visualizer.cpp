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
// @file land_mark_visualizer.cpp
// @brief This file contains a ros node allowing to visualize land mark objects
//

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <antikythera_msgs/msg/land_mark_object.hpp>
// Antikythera includes
#include "land_mark_object.hpp"
#include "land_mark_feature.hpp"
#include "point_cloud_feature.hpp"

class LandMarkVisualizer : public rclcpp::Node
{
public:
  // constructor
  LandMarkVisualizer()
  : Node("landMarkVisualizer"),
    vis2_("LandMark Visualizer")
  {
    // create subscriber
    subscriber_ = this->create_subscription<antikythera_msgs::msg::LandMarkObject>(
      "/antikythera/land_mark_object", 10, std::bind(
        &LandMarkVisualizer::on_land_mark_received, this, std::placeholders::_1
    ));
    vis2_.setBackgroundColor(0.0, 0.0, 0.0);  // black background
    vis2_.initCameraParameters();  // Initialize camera
  }

private:
  rclcpp::Subscription<antikythera_msgs::msg::LandMarkObject>::SharedPtr subscriber_;
  pcl::visualization::PCLVisualizer vis2_;

  void on_land_mark_received(const antikythera_msgs::msg::LandMarkObject::SharedPtr msg)
  {
    printf("Received LandMarkObject with ID: %d\n", msg->id);

    antikythera::LandMarkObject land_mark_object(msg->id);
    antikythera::LandMarkObject::fromROSMsg(*msg, land_mark_object);
    land_mark_object.print();
    auto features = land_mark_object.get_features_object<antikythera::PointCloudFeature>();
    // print features
    for (const auto & feature : features) {
      feature->print();
    }

    for (const auto & featureObject : land_mark_object.get_features_object
      <antikythera::PointCloudFeature>())
    {
      // Ensure featureObject is not null
      if (!featureObject) {
        std::cerr << "Warning: Null feature object encountered!" << std::endl;
        continue;
      }

      // Extract cloud from feature
      auto cloud = std::static_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(
        featureObject->get_feature_data());

      if (!cloud) {
        std::cerr << "Warning: Null point cloud feature data!" << std::endl;
        continue;
      }

      // convert to RGB for visualization
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*cloud, *cloud_rgb);

      // add point cloud to visualizer
      vis2_.addPointCloud(cloud_rgb, "cloud_" + std::to_string(msg->id));
    }
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto visualizer_node = std::make_shared<LandMarkVisualizer>();
  rclcpp::spin(visualizer_node);
  rclcpp::shutdown();
  return 0;
}
