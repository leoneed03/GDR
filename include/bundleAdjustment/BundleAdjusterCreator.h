//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEADJUSTERCREATOR_H
#define GDR_BUNDLEADJUSTERCREATOR_H

#include <memory>

#include "bundleAdjustment/BundleAdjuster.h"

#include "cameraModel/CameraRGBD.h"
#include "keyPoints/KeyPointInfo.h"

namespace gdr {

    class BundleAdjusterCreator {

    public:

        BundleAdjusterCreator() = delete;

        enum class BundleAdjustmentType {
            USE_DEPTH_INFO
        };

        static std::unique_ptr<BundleAdjuster> getBundleAdjuster(const BundleAdjustmentType &bundleAdjustmentType);
    };
}

#endif
