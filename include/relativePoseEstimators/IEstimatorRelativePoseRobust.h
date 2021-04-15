//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IESTIMATORRELATIVEPOSEROBUST_H
#define GDR_IESTIMATORRELATIVEPOSEROBUST_H

#include "ParamsRANSAC.h"
#include "parametrization/SE3.h"
#include "cameraModel/CameraRGBD.h"

namespace gdr {


    /** Robust relative pose estimator for 2 given point clouds */
    class IEstimatorRelativePoseRobust {

    public:
        /**
         * Estimate SE3 transformation between point clouds observed by two different cameras
         *
         * @param toBeTransformedPoints, destinationPoints 3D point clouds observed by first and second camera
         *      each column of matrix represents one point: {x, y, z, 1.0}
         * @param cameraIntrToBeTransformed, cameraIntrDestination store camera intrinsics
         * @param[out] estimationSuccess indicates ransac procedure success
         * @param[out] inlierIndices contains numbers of points that represent "inlier" matches
         * @returns relative pose estimation SE3 for given pair of point clouds (id by default)
         */
        virtual SE3 estimateRelativePose(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                bool &estimationSuccess,
                std::vector<int> &inlierIndices) = 0;

        virtual ~IEstimatorRelativePoseRobust() = default;
    };
}

#endif
