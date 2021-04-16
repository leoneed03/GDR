//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

namespace gdr {

    RotationMeasurement::RotationMeasurement(const SO3 &relativeRotationToSet,
                                             int newIndexFrom,
                                             int newIndexTo) : relativeRotationQuat(relativeRotationToSet),
                                                               indexFromDestination(newIndexFrom),
                                                               indexToToBeTransformed(newIndexTo) {}


    const SO3 &RotationMeasurement::getRotationSO3() const {
        return relativeRotationQuat;
    }


    int RotationMeasurement::getIndexFromDestination() const {
        return indexFromDestination;
    }

    int RotationMeasurement::getIndexToToBeTransformed() const {
        return indexToToBeTransformed;
    }
}