//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONROBUSTOPTIMIZERCREATOR_H
#define GDR_ROTATIONROBUSTOPTIMIZERCREATOR_H

#include <memory>

#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"
#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

namespace gdr {
    class RotationRobustOptimizerCreator {
    public:

        RotationRobustOptimizerCreator() = delete;

        enum class RobustParameterType {
            DEFAULT
        };

        static std::unique_ptr<RotationRobustOptimizer> getRefiner(const RobustParameterType &robustParameterType);
    };
}
#endif
