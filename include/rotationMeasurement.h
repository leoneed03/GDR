//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONMEASUREMENT_H
#define GDR_ROTATIONMEASUREMENT_H

#include <Eigen/Eigen>

#include "Rotation3d.h"

namespace gdr {
    struct rotationMeasurement {

        Rotation3d relativeRotationQuat;
        int indexFrom;
        int indexTo;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        rotationMeasurement(const Eigen::Quaterniond& newRelativeRotationQuat, int newIndexFrom, int newIndexTo);
        Eigen::Quaterniond getRotationQuat() const;
        const Rotation3d& getRotation3d() const;

        int getIndexFromToBeTransformed() const;
        int getIndexToDestination() const;

    };
}

#endif
