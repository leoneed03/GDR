//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEADJUSTERCREATOR_H
#define GDR_BUNDLEADJUSTERCREATOR_H

#include <memory>

#include "bundleAdjustment/IBundleAdjuster.h"

#include "parametrization/cameraRGBD.h"
#include "keyPoints/KeyPointInfo.h"

namespace gdr {

    class BundleAdjusterCreator {

    public:

        BundleAdjusterCreator() = delete;

        enum class BundleAdjustmentType {USE_DEPTH_INFO};

        static std::unique_ptr<IBundleAdjuster> getFeatureDetector(const std::vector<Point3d> &points,
                                                                   const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                                                                   const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointinfo,
                                                                   const BundleAdjustmentType& bundleAdjustmentType);
    };
}

#endif
