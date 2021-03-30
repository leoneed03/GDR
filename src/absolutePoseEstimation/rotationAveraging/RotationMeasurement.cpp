//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

namespace gdr {

    RotationMeasurement::RotationMeasurement(const SO3 &relativeRotationToSet,
                                             int newIndexFrom,
                                             int newIndexTo) : relativeRotationQuat(relativeRotationToSet),
                                                               indexFrom(newIndexFrom),
                                                               indexTo(newIndexTo) {}


    const SO3 &RotationMeasurement::getRotationSO3() const {
        return relativeRotationQuat;
    }


    int RotationMeasurement::getIndexFromToBeTransformed() const {
        return indexFrom;
    }

    int RotationMeasurement::getIndexToDestination() const {
        return indexTo;
    }
}