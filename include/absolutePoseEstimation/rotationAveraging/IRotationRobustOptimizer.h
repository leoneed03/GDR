//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IROTATIONROBUSTOPTIMIZER_H
#define GDR_IROTATIONROBUSTOPTIMIZER_H

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "parametrization/SO3.h"

namespace gdr {

    class IRotationRobustOptimizer {
    public:
        /**
         * Use robust kernel to optimize orientations using results of global averaging as initestimation
         *
         * @param orientations contains information about globally optimal "averaged" orientations
         * @param pairwiseRelRotations relative rotations
         * @param indexFixedPose pose which orientation is Id
         *
         * @returns vector of optimized orientations
         */
        virtual std::vector<SO3> getOptimizedOrientation(
                const std::vector<SO3> &orientations,
                const std::vector<RotationMeasurement> &pairwiseRelRotations,
                int indexFixedPose) = 0;

        virtual ~IRotationRobustOptimizer() = default;
    };
}

#endif
