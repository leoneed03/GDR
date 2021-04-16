//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONMEASUREMENT_H
#define GDR_ROTATIONMEASUREMENT_H

#include <Eigen/Eigen>

#include "parametrization/SO3.h"

namespace gdr {

    class RotationMeasurement {

        SO3 relativeRotationQuat;

        int indexFromDestination;
        int indexToToBeTransformed;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RotationMeasurement(const SO3 &relativeRotation,
                            int indexFromDestination,
                            int indexToToBeTransformed);

        const SO3 &getRotationSO3() const;

        int getIndexFromDestination() const;

        int getIndexToToBeTransformed() const;

    };
}

#endif
