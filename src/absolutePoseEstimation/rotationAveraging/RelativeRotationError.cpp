//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RelativeRotationError.h"

namespace gdr {

    RelativeRotationError::RelativeRotationError(const SO3 &relativeRotationToSet) :
            relativeRotation(relativeRotationToSet) {}

    ceres::CostFunction *RelativeRotationError::Create(const SO3 &relativeRotationToSet) {
        return (new ceres::AutoDiffCostFunction<RelativeRotationError, 3, 4, 4>(
                new RelativeRotationError(relativeRotationToSet)));
    }

}