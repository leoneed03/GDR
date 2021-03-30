//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESTIMATOR3POINTS_H
#define GDR_ESTIMATOR3POINTS_H

#include <Eigen/Eigen>

#include "parametrization/cameraRGBD.h"
#include "parametrization/SE3.h"

namespace gdr {

    class Estimator3Points {

    public:
        virtual SE3
        getRt(const Eigen::Matrix4Xd &toBeTransormedPoints,
              const Eigen::Matrix4Xd &destinationPoints,
              const CameraRGBD &cameraIntrToBeTransformed,
              const CameraRGBD &cameraIntrDestination) const;

    };
}

#endif
