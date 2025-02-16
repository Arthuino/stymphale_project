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
// @file land_mark_object.cpp
// @brief This file contains the implementation of the LandMarkObject class
//

#include "land_mark_object.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <string>
#include <memory>

#include "land_mark_feature.hpp"

// TODO : Implement the toROSMsg and fromROSMsg method

namespace antikythera {
    
    // Constructors
    LandMarkObject::LandMarkObject(int id) 
        : id(id), label("default") {}

    LandMarkObject::LandMarkObject(int id, const std::string& label) 
        : id(id), label(label) {}

    // Print
    void LandMarkObject::print() const {
        std::cout << "LandMarkObject ID: " << id << ", Label: " << label << std::endl;
        std::cout << "Features: " << features.size() << std::endl;
        for (const auto& feature : features) {
            feature->print();
        }
    }

    // Getters
    int LandMarkObject::get_id() const noexcept {
        return id;
    }

    const std::string& LandMarkObject::get_label() const noexcept {
        return label;
    }

    const std::vector<std::shared_ptr<LandMarkFeature>>& LandMarkObject::get_features() const noexcept {
        return features;
    }

    // Setters
    void LandMarkObject::set_label(const std::string& label) {
        this->label = label;
    }

    void LandMarkObject::add_feature(std::shared_ptr<LandMarkFeature> feature) {
        features.push_back(std::move(feature));
    }

    void LandMarkObject::remove_feature(size_t index) {
        if (index < features.size()) {
            features.erase(features.begin() + index);
        } else {
            std::cerr << "Error: Feature index out of bounds." << std::endl;
        }
    }


    void LandMarkObject::toROSMsg(const LandMarkObject& land_mark_object, antikythera_msgs::msg::LandMarkObject& msg) {
        msg.id = land_mark_object.get_id();
        msg.label = land_mark_object.get_label();
        for (const auto& feature : land_mark_object.get_features()) {
            antikythera_msgs::msg::LandMarkFeature feature_msg;
            LandMarkFeature::toROSMsg(*feature, feature_msg);
            msg.features.push_back(feature_msg);
        }
    }
    
    void LandMarkObject::fromROSMsg(const antikythera_msgs::msg::LandMarkObject& msg, LandMarkObject& land_mark_object){
        land_mark_object = LandMarkObject(msg.id, msg.label);
        for (const auto& feature_msg : msg.features) {
            std::shared_ptr<LandMarkFeature> feature = std::make_shared<LandMarkFeature>();
            LandMarkFeature::fromROSMsg(feature_msg, *feature);
            land_mark_object.add_feature(feature);
        }
    }
    
} // namespace antikythera