//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/Estimator3Points.h"
#include <iostream>

#include <Eigen/Eigen>

namespace gdr {

    void Estimator3Points::hello() {
        std::cout << "hello" << std::endl;
    }

    SE3 Estimator3Points::getRt(const Eigen::Matrix4Xd &toBeTransformed3Points,
                                const Eigen::Matrix4Xd &dest3Points,
                                const CameraRGBD &cameraIntrToBeTransformed,
                                const CameraRGBD &cameraIntrDestination) const {
        int dim = 3;
        int minNumPoints = 3;
        int numPoints = toBeTransformed3Points.cols();
        assert(numPoints == minNumPoints);
        assert(numPoints == dest3Points.cols());

        Eigen::Matrix4d umeyama3p = umeyama(toBeTransformed3Points.block(0, 0, dim, dim),
                                            dest3Points.block(0, 0, dim, dim));
        return SE3(umeyama3p);
    }
}