//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_BUNDLEADJUSTER_H
#define GDR_BUNDLEADJUSTER_H

#include "parametrization/SE3.h"
#include "parametrization/Point3d.h"
#include "cameraModel/CameraRGBD.h"
#include "keyPoints/KeyPointInfo.h"
#include <unordered_map>

namespace gdr {

    /** Interface for nonlinear poses and keypoints optimization*/
    class BundleAdjuster {
    public:
        /**
         *
         * @param points contains observed points predicted coordinates
         * @param absolutePoses contains information about cameras predicted SE3 poses and camera intrinsic parameters
         * @param keyPointInfo for each pose maps from obseerved keypoint index to this keypoint parameters
         * @param fixedPoseNumber is the number of the pose with zero coordinates
         *
         * @returns vector of optimized poses
         */
        virtual std::vector<SE3> optimizePointsAndPoses(const std::vector<Point3d> &points,
                                                        const std::vector<std::pair<SE3, CameraRGBD>> &absolutePoses,
                                                        const std::vector<std::unordered_map<int, KeyPointInfo>> &keyPointInfo,
                                                        int fixedPoseNumber) = 0;

        virtual std::vector<Point3d> getOptimizedPoints() const = 0;

        virtual ~BundleAdjuster() = default;
    };
}

#endif
