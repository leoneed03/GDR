//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_INLIERCOUNTER_H
#define GDR_INLIERCOUNTER_H

#include <vector>
#include "Eigen/Eigen"

#include "parametrization/cameraRGBD.h"
#include "parametrization/SE3.h"
#include "ParamsRANSAC.h"

namespace gdr {
    class InlierCounter {

        // return vector of pairs for each inlier:
        // {errorInPixelsAfterTransformationAndProjection, columnNumber in Original Matrix toBeTransformedPoints}
    public:
        virtual std::vector<std::pair<double, int>> calculateProjectionErrors(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntr3x3Destination,
                const SE3 &umeyamaRt,
                const ParamsRANSAC &paramsRansac) const;
    };
}

#endif
