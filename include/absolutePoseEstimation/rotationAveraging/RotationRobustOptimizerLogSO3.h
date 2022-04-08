//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONROBUSTOPTIMIZERLOGSO3_H
#define GDR_ROTATIONROBUSTOPTIMIZERLOGSO3_H

#include <map>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "parametrization/SO3.h"

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"
#include "absolutePoseEstimation/rotationAveraging/RelativeRotationError.h"

namespace gdr {

    class RotationRobustOptimizerLogSO3 : public RotationRobustOptimizer {

        std::vector<SO3> orientations;
        std::vector<RotationMeasurement> relativeRotations;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<SO3> getOptimizedOrientation(const std::vector<SO3> &orientations,
                                                 const std::vector<RotationMeasurement> &pairWiseRotations,
                                                 int indexFixed) override;
    };
}

#endif
