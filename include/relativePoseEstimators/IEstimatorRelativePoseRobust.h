//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IESTIMATORRELATIVEPOSEROBUST_H
#define GDR_IESTIMATORRELATIVEPOSEROBUST_H

#include "ParamsRANSAC.h"
#include "parametrization/SE3.h"
#include "parametrization/cameraRGBD.h"

namespace gdr {

    // robust relative pose estimator min number of matches for pose estimation is 3
    class IEstimatorRelativePoseRobust {
    public:
        virtual SE3 estimateRelativePose(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntrToBeTransformed,
                const CameraRGBD &cameraIntrDestination,
                const ParamsRANSAC &paramsRansac,
                bool& estimationSuccess,
                std::vector<int>& inlierIndices) = 0;
        virtual ~IEstimatorRelativePoseRobust() = default;
    };
}

#endif
