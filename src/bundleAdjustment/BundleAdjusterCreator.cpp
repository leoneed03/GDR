//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "bundleAdjustment/BundleAdjusterCreator.h"
#include "bundleAdjustment/BundleAdjuster.h"

namespace gdr {

    std::unique_ptr<IBundleAdjuster>
    BundleAdjusterCreator::getFeatureDetector(const std::vector<Point3d> &points,
                                              const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                                              const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo,
                                              const BundleAdjusterCreator::BundleAdjustmentType &bundleAdjustmentType) {

        if (bundleAdjustmentType == BundleAdjustmentType::USE_DEPTH_INFO) {

        } else {
            std::cout << "only BA with depth info is implemented" << std::endl;
        }

        return std::make_unique<BundleAdjuster>(points,
                                                absolutePoses,
                                                keyPointinfo);
    }
}