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

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv2/core.hpp>

using namespace std::chrono_literals;

class LandMarkSlicer : public rclcpp::Node
{
public:
  LandMarkSlicer()
  : Node("landMarkSlicer")
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/antikythera/emulator/point_cloud", 10, std::bind(
        &LandMarkSlicer::sub_callback, this, std::placeholders::_1
    ));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

  void sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    printf("Received point cloud\n");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    segmentation(cloud);
  }

  void segmentation(const pcl::PointCloud<pcl::PointXYZ> fullcloud)
  {
    printf("Segmentation\n");
    printf("Full cloud size: %ld\n", fullcloud.size());
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto slicer_node = std::make_shared<LandMarkSlicer>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slicer_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
