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

using antikythera::PointCloudFeature;
using antikythera::LandMarkFeature;
using antikythera_msgs::msg::LandMarkFeature as LandMarkFeatureMsg;

TEST(PointCloudFeatureTest, ConstructorTest) {
    PointCloudFeature feature;
    EXPECT_EQ(feature.get_feature_type(), FEATURE_TYPE_POINT_CLOUD);
}

TEST(PointCloudFeatureTest, PrintTest) {
    PointCloudFeature feature;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    auto feature_data = std::make_shared<std::any>(cloud);
    feature.set_feature(feature_data);

    testing::internal::CaptureStdout();
    feature.print();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "PointCloudFeature with 1 points.\n");
}

TEST(PointCloudFeatureTest, GetFeatureDataTest) {
    PointCloudFeature feature;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    auto feature_data = std::make_shared<std::any>(cloud);
    feature.set_feature(feature_data);

    auto retrieved_data = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>(
                                                                        feature.get_feature_data()
                                                                          );
    EXPECT_EQ(retrieved_data->size(), 1);
    EXPECT_EQ((*retrieved_data)[0].x, 1.0);
    EXPECT_EQ((*retrieved_data)[0].y, 2.0);
    EXPECT_EQ((*retrieved_data)[0].z, 3.0);
}

TEST(PointCloudFeatureTest, SetFeatureDataInvalidTypeTest) {
    PointCloudFeature feature;
    auto invalid_data = std::make_shared<std::any>(std::make_shared<int>(42));

    testing::internal::CaptureStderr();
    feature.set_feature(invalid_data);
    std::string output = testing::internal::GetCapturedStderr();
    EXPECT_EQ(output, "Error: feature_data is not a valid PointCloud pointer!\n");
}

TEST(PointCloudFeatureTest, SetFeatureDataValidTypeTest) {
    PointCloudFeature feature;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    auto feature_data = std::make_shared<std::any>(cloud);

    feature.set_feature(feature_data);
    auto retrieved_data = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>(
                                                                        feature.get_feature_data()
                                                                          );
    EXPECT_EQ(retrieved_data->size(), 1);
    EXPECT_EQ((*retrieved_data)[0].x, 1.0);
    EXPECT_EQ((*retrieved_data)[0].y, 2.0);
    EXPECT_EQ((*retrieved_data)[0].z, 3.0);
}

TEST(PointCloudFeatureTest, ToROSMsgTest) {
    auto feature = std::make_shared<PointCloudFeature>();
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    auto feature_data = std::make_shared<std::any>(cloud);
    feature->set_feature(feature_data);

    LandMarkFeatureMsg msg;
    LandMarkFeature::toROSMsg(feature, msg);

    EXPECT_EQ(msg.feature_type, FEATURE_TYPE_POINT_CLOUD);
    EXPECT_TRUE(msg.is_point_cloud);
    EXPECT_EQ(msg.point_cloud.width, 1);
    EXPECT_EQ(msg.point_cloud.height, 1);
}

TEST(PointCloudFeatureTest, FromROSMsgTest) {
    LandMarkFeatureMsg msg;
    msg.is_point_cloud = true;
    msg.feature_type = FEATURE_TYPE_POINT_CLOUD;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    pcl::toROSMsg(cloud, msg.point_cloud);

    std::shared_ptr<LandMarkFeature> feature;
    LandMarkFeature::fromROSMsg(msg, feature);

    auto pointCloudFeature = std::dynamic_pointer_cast<PointCloudFeature>(feature);
    ASSERT_TRUE(pointCloudFeature != nullptr);

    auto feature_data = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>(
                                                              pointCloudFeature->get_feature_data()
                                                              );
    EXPECT_EQ(feature_data->size(), 1);
    EXPECT_EQ((*feature_data)[0].x, 1.0);
    EXPECT_EQ((*feature_data)[0].y, 2.0);
    EXPECT_EQ((*feature_data)[0].z, 3.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
