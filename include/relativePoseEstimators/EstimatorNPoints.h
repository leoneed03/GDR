//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESTIMATORNPOINTS_H
#define GDR_ESTIMATORNPOINTS_H

#include "parametrization/SE3.h"
#include "parametrization/cameraRGBD.h"

namespace gdr {

    class EstimatorNPoints {

    public:
        virtual SE3
        getRt(const Eigen::Matrix4Xd &toBeTransormedPoints,
              const Eigen::Matrix4Xd &destinationPoints,
              const CameraRGBD &cameraIntrToBeTransformed,
              const CameraRGBD &cameraIntrDestination) const;
    };
}

#endif
