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
// @file point_cloud_feature_test.cpp
// @brief This file contains tests for the PointCloudFeature class
// inherited classes from LandMarkFeature
//

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>

#include "point_cloud_feature.hpp"
#include "land_mark_feature.hpp"
// Antikythera message
#include "antikythera_msgs/msg/land_mark_object.hpp"

using antikythera::PointCloudFeature;
using antikythera::LandMarkFeature;
using LandMarkFeatureMsg = antikythera_msgs::msg::LandMarkFeature;

// HELPER FUNCTIONS
pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud(const std::vector<pcl::PointXYZ> & points)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : points) {
    cloud->push_back(point);
  }
  return cloud;
}

void validatePointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::vector<pcl::PointXYZ> & expectedPoints)
{
  ASSERT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->size(), expectedPoints.size());
  for (size_t i = 0; i < expectedPoints.size(); ++i) {
    EXPECT_FLOAT_EQ((*cloud)[i].x, expectedPoints[i].x);
    EXPECT_FLOAT_EQ((*cloud)[i].y, expectedPoints[i].y);
    EXPECT_FLOAT_EQ((*cloud)[i].z, expectedPoints[i].z);
  }
}

// TESTS
TEST(PointCloudFeatureTest, ConstructorTest) {
  PointCloudFeature feature;
  EXPECT_EQ(feature.get_feature_type(), antikythera::FEATURE_TYPE_POINT_CLOUD);
}

TEST(PointCloudFeatureTest, SetFeatureDataValidTest) {
  // ARRANGE - Create the feature object
  PointCloudFeature feature;
  auto cloud = createPointCloud({pcl::PointXYZ(1.0, 2.0, 3.0)});

  // ACT - Set the feature data
  feature.set_feature_data(cloud);

  // ASSERT - Retrieve the feature data and test it
  auto retrieved_cloud = feature.get_feature_data<pcl::PointCloud<pcl::PointXYZ>>();
  validatePointCloud(retrieved_cloud, {pcl::PointXYZ(1.0, 2.0, 3.0)});
}

TEST(PointCloudFeatureTest, PrintTest) {
  // Create a feature object and set the data
  PointCloudFeature feature;
  auto cloud = createPointCloud({pcl::PointXYZ(1.0, 2.0, 3.0)});
  feature.set_feature_data(cloud);

  // Capture and verify the printed output
  testing::internal::CaptureStdout();
  feature.print();
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output, "PointCloudFeature with 1 points.\n");
}

TEST(PointCloudFeatureTest, ToROSMsgTest) {
  // ARRANGE - Create the feature object
  auto feature = std::make_shared<PointCloudFeature>();
  auto cloud = createPointCloud({pcl::PointXYZ(1.0, 2.0, 3.0)});
  feature->set_feature_data(cloud);

  // ACT - Convert the feature to a ROS message
  LandMarkFeatureMsg msg;
  msg = feature->toROSMsg();

  // ASSERT - Verify the message contents
  EXPECT_EQ(msg.feature_type, antikythera::FEATURE_TYPE_POINT_CLOUD);
  EXPECT_TRUE(msg.is_point_cloud);
  EXPECT_EQ(msg.point_cloud.width, 1);
  EXPECT_EQ(msg.point_cloud.height, 1);
}

TEST(PointCloudFeatureTest, FromROSMsgValidTest) {
  // ARRANGE - Create a ROS message with a point cloud
  LandMarkFeatureMsg msg;
  msg.is_point_cloud = true;
  msg.feature_type = antikythera::FEATURE_TYPE_POINT_CLOUD;
  auto cloud = createPointCloud({pcl::PointXYZ(1.0, 2.0, 3.0)});
  pcl::toROSMsg(*cloud, msg.point_cloud);

  // ACT - Create a feature object from the ROS message
  std::shared_ptr<LandMarkFeature> feature = std::make_shared<PointCloudFeature>();
  feature->fromROSMsg(msg);

  auto feature_data = feature->get_feature_data<pcl::PointCloud<pcl::PointXYZ>>();
  validatePointCloud(feature_data, {pcl::PointXYZ(1.0, 2.0, 3.0)});
}

TEST(PointCloudFeatureTest, FromROSMsgInvalidTest) {
  LandMarkFeatureMsg msg;
  msg.is_point_cloud = false;
  msg.feature_type = antikythera::FEATURE_TYPE_POINT_CLOUD;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  pcl::toROSMsg(cloud, msg.point_cloud);

  std::shared_ptr<LandMarkFeature> feature = std::make_shared<PointCloudFeature>();
  testing::internal::CaptureStderr();
  feature->fromROSMsg(msg);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_EQ(output, "Error: Data from ROS msg is not a point cloud\n");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
