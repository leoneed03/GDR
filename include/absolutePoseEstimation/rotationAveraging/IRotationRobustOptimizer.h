//
// Created by leo on 3/29/21.
//

#ifndef GDR_IROTATIONROBUSTOPTIMIZER_H
#define GDR_IROTATIONROBUSTOPTIMIZER_H

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "parametrization/SO3.h"

namespace gdr {
    class IRotationRobustOptimizer {
    public:

        virtual std::vector<SO3> getOptimizedOrientation(
                const std::vector<SO3> &orientations,
                const std::vector<RotationMeasurement> &pairwiseRelRotations,
                int indexFixedPose = 0) = 0;

        virtual ~IRotationRobustOptimizer() = default;
    };
}

#endif
