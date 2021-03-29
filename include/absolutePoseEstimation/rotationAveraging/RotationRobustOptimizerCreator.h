//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONROBUSTOPTIMIZERCREATOR_H
#define GDR_ROTATIONROBUSTOPTIMIZERCREATOR_H

#include <memory>

#include "absolutePoseEstimation/rotationAveraging/IRotationRobustOptimizer.h"
#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

namespace gdr {
    class RotationRobustOptimizerCreator {
    public:

        RotationRobustOptimizerCreator() = delete;

        enum class RobustParameterType {
            DEFAULT
        };

        static std::unique_ptr<IRotationRobustOptimizer> getRefiner(const std::vector<SO3> &orientations,
                                                                    const std::vector<RotationMeasurement> &pairWiseRotations,
                                                                    const RobustParameterType &robustParameterType);
    };
}
#endif
