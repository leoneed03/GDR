//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "bundleAdjustment/BundleAdjusterCreator.h"
#include "bundleAdjustment/BundleAdjuster.h"

namespace gdr {

    std::unique_ptr<IBundleAdjuster>
    BundleAdjusterCreator::getBundleAdjuster(const BundleAdjusterCreator::BundleAdjustmentType &bundleAdjustmentType) {

        if (bundleAdjustmentType == BundleAdjustmentType::USE_DEPTH_INFO) {

        } else {
            std::cout << "only BA with depth info is implemented" << std::endl;
        }

        return std::make_unique<BundleAdjuster>();
    }
}