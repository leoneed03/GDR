//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/EstimatorNPoints.h"

namespace gdr {

    SE3 EstimatorNPoints::getRt(const Eigen::Matrix4Xd &toBeTransformedPoints,
                                const Eigen::Matrix4Xd &destinationPoints,
                                const CameraRGBD &cameraIntrToBeTransformed,
                                const CameraRGBD &cameraIntrDestination) const {
        int dim = 3;
        int numPoints = toBeTransformedPoints.cols();
        int minNumPoints = 3;

        assert(numPoints >= minNumPoints);
        assert(numPoints == destinationPoints.cols());

        Eigen::Matrix4d umeyamaNp = umeyama(
                toBeTransformedPoints.block(0, 0, dim, numPoints),
                destinationPoints.block(0, 0, dim, numPoints));

        return SE3(umeyamaNp);
    }
}