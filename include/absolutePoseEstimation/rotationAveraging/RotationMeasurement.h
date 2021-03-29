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

        int indexFrom;
        int indexTo;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RotationMeasurement(const SO3 &newRelativeRotationQuat,
                            int newIndexFrom,
                            int newIndexTo);

        Eigen::Quaterniond getRotationQuat() const;

        const SO3 &getRotation3d() const;

        int getIndexFromToBeTransformed() const;

        int getIndexToDestination() const;

    };
}

#endif