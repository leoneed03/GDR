//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerCreator.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"

namespace gdr {

    std::unique_ptr<IRotationRobustOptimizer> RotationRobustOptimizerCreator::getRefiner(
            const RotationRobustOptimizerCreator::RobustParameterType &robustParameterType) {

        if (robustParameterType == RobustParameterType::DEFAULT) {

        } else {
            std::cout << "only default ceres optimization is available at the moment" << std::endl;
        }

        return std::make_unique<RotationRobustOptimizer>();
    }
}
