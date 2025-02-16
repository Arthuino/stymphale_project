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

#ifndef LAND_MARK_FEATURE_HPP
#define LAND_MARK_FEATURE_HPP

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <string>
#include <iostream>
#include <variant>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <antikythera_msgs/msg/land_mark_feature.hpp>

namespace antikythera {

    // Alias for variant holding possible feature types
    using FeatureData = std::variant<
        pcl::PointCloud<pcl::PointXYZ>::Ptr,     // PointCloud
        geometry_msgs::msg::TransformStamped   // ROS TransformStamped
    >;

    // Abstract class definition for a feature of a LandMarkObject
    class LandMarkFeature {
    public:

        virtual ~LandMarkFeature();

        // void constructor
        LandMarkFeature();

        LandMarkFeature(const std::string& feature_type);

        LandMarkFeature(const std::string& feature_type, FeatureData feature_data);

        virtual void print() const;  // Print details of the feature

        // Feature Data
        void set_feature(FeatureData feature);
        FeatureData get_feature() const;

        // Feature type
        std::string get_feature_type() const;

        // ROS message conversion methods
        static void toROSMsg(const LandMarkFeature& landMarkFeature, antikythera_msgs::msg::LandMarkFeature& msg);
        static void fromROSMsg(const antikythera_msgs::msg::LandMarkFeature& msg, LandMarkFeature& landMarkFeature);

    protected:
        std::string feature_type;
        FeatureData feature_data;
    };
} // namespace antikythera

#endif // LAND_MARK_FEATURE_HPP