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
// @file land_mark_object.hpp
// @brief This file contains the header of the LandMarkObject class
//

#ifndef LAND_MARK_OBJECT_HPP_
#define LAND_MARK_OBJECT_HPP_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <string>
#include <vector>
#include <memory>

#include "land_mark_feature.hpp"
// message includes
#include "antikythera_msgs/msg/land_mark_object.hpp"

namespace antikythera
{
class LandMarkObject
{
public:
  // CONSTRUCTORS
  LandMarkObject() = default;
  explicit LandMarkObject(int id);
  LandMarkObject(int id, const std::string & label);

  // PRINT
  void print() const;


  // Getters
  [[nodiscard]] int get_id() const noexcept;
  [[nodiscard]] const std::string & get_label() const noexcept;
  
  /*
   * @brief Get all features of a specific type
   * @tparam FeatureType The type of feature to get (e.g. PointCloudFeature)
   * 
   * @return A vector of shared pointers to the features of the specified type
   */
  template<typename FeatureType>  // FeatureType can be PointCloudFeature, etc.
  [[nodiscard]] std::vector<std::shared_ptr<FeatureType>> get_features_object() const noexcept
  {
    std::vector<std::shared_ptr<FeatureType>> specific_features;

    for (const auto & feature : features)
    {
        if (auto casted_feature = std::dynamic_pointer_cast<FeatureType>(feature))
        {
            specific_features.push_back(casted_feature);
        }
    }
    return specific_features;
  }

  // Setters
  void set_label(const std::string & label);
  void add_feature_object(std::shared_ptr<LandMarkFeature> feature);
  void remove_feature_object(size_t index);  // Removes a feature by index

  // ROS serialization methods
  static void toROSMsg(
    const LandMarkObject & land_mark_object,
    antikythera_msgs::msg::LandMarkObject & msg);

  static void fromROSMsg(
    const antikythera_msgs::msg::LandMarkObject & msg,
    LandMarkObject & land_mark_object);

private:
  int id;
  std::string label;
  std::vector<std::shared_ptr<LandMarkFeature>> features;  // Store multiple feature types
};
}  // namespace antikythera

#endif  // LAND_MARK_OBJECT_HPP_
