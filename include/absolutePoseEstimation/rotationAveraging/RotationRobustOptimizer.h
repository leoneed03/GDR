//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONROBUSTOPTIMIZER_H
#define GDR_ROTATIONROBUSTOPTIMIZER_H

#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "parametrization/SO3.h"

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "absolutePoseEstimation/rotationAveraging/IRotationRobustOptimizer.h"
#include "absolutePoseEstimation/rotationAveraging/RelativeRotationError.h"

namespace gdr {

    class RotationRobustOptimizer : public IRotationRobustOptimizer {

        std::vector<SO3> orientations;
        std::vector<RotationMeasurement> relativeRotations;
        bool printProgressToConsole = false;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<SO3> getOptimizedOrientation(const std::vector<SO3> &orientations,
                                                 const std::vector<RotationMeasurement> &pairWiseRotations,
                                                 int indexFixed = 0) override;

        bool getPrintToConsole() const;

        void setPrintProgressToConsole(bool printToConsoleToSet);
    };
}

#endif
