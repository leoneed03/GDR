//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONMEASUREMENT_H
#define GDR_ROTATIONMEASUREMENT_H

#include <Eigen/Eigen>

namespace gdr {
    struct rotationMeasurement {

        Eigen::Quaterniond relativeRotationQuat;
        int indexFrom;
        int indexTo;

        rotationMeasurement(const Eigen::Quaterniond& newRelativeRotationQuat, int newIndexFrom, int newIndexTo);

    };
}

#endif
