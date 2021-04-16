//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONAVERAGER_H
#define GDR_ROTATIONAVERAGER_H

#include <string>
#include <vector>
#include <Eigen/Eigen>

#include "parametrization/SO3.h"
#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

namespace gdr {

    class RotationAverager {

    public:
        static std::vector<SO3>
        shanonAveraging(
                const std::vector<RotationMeasurement> &relativeRotations,
                const std::string &pathToRelativeRotationsOut,
                int maxDimension = 10,
                bool printProgressToConsole = false);

    };
}

#endif
