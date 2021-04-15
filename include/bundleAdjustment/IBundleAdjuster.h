//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IBUNDLEADJUSTER_H
#define GDR_IBUNDLEADJUSTER_H

#include "parametrization/SE3.h"
#include "parametrization/Point3d.h"
#include "cameraModel/CameraRGBD.h"
#include "keyPoints/KeyPointInfo.h"
#include <unordered_map>

namespace gdr {

    class IBundleAdjuster {
    public:
        virtual std::vector<SE3> optimizePointsAndPoses(const std::vector<Point3d> &points,
                                                        const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                                                        const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo,
                                                        int fixedPoseNumber = 0) = 0;

        virtual std::vector<Point3d> getOptimizedPoints() const = 0;

        virtual ~IBundleAdjuster() = default;
    };
}

#endif
