#include "point_cloud_feature.hpp"
#include <iostream>
#include <variant>

#include "land_mark_feature.hpp"
#include "point_cloud_feature.hpp"
#include <iostream>
#include <variant>

namespace antikythera {

    void PointCloudFeature::print() const {
        if (auto* pc = std::get_if<pcl::PointCloud<pcl::PointXYZ>::Ptr>(&feature_data)) {
            std::cout << "PointCloudFeature with " << (*pc)->size() << " points." << std::endl;
        } else {
            std::cout << "Feature is not a PointCloud." << std::endl;
        }
    }

    void PointCloudFeature::set_feature(FeatureData feature) {
        // Set the feature directly using the parent class's method
        LandMarkFeature::set_feature(feature);
    }

    FeatureData PointCloudFeature::get_feature() const {
        // Return the feature data directly from the parent class
        return LandMarkFeature::get_feature();
    }

    std::string PointCloudFeature::get_feature_type() const {
        return "PointCloud";
    }

} // namespace antikythera