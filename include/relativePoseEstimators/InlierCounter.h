//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_INLIERCOUNTER_H
#define GDR_INLIERCOUNTER_H

#include <vector>

#include "Eigen/Eigen"

#include "cameraModel/CameraRGBD.h"
#include "parametrization/SE3.h"
#include "ParamsRANSAC.h"

namespace gdr {

    class InlierCounter {

    public:
        /** Calculate reprojection errors for alignment
         * @param toBeTransformedPoints point cloud to be aligned
         * @param destinationPoints static point cloud
         * @param cameraIntr3x3Destination camera intrinsics for destination camera
         * @param umeyamaRt SE3 transformation of toBeTransformed point cloud
         * @param paramsRansac LoRANSAC parameters
         * @returns all inlier reprojection errors where each element contains reprojection error and point's index
         */
        virtual std::vector<std::pair<double, int>> calculateInlierProjectionErrors(
                const Eigen::Matrix4Xd &toBeTransformedPoints,
                const Eigen::Matrix4Xd &destinationPoints,
                const CameraRGBD &cameraIntr3x3Destination,
                const SE3 &umeyamaRt,
                const ParamsRANSAC &paramsRansac) const;

        virtual ~InlierCounter() = default;
    };
}

#endif
