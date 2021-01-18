//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "translationMeasurement.h"

namespace gdr {

    translationMeasurement::translationMeasurement(const Eigen::Vector3d &newTranslation3d, int newIndexFrom,
                                                   int newIndexTo) : translation3d(newTranslation3d),
                                                                     indexFrom(newIndexFrom),
                                                                     indexTo(newIndexTo) {}
    Eigen::Vector3d translationMeasurement::getTranslation() const {
        return translation3d;
    }


    int translationMeasurement::getIndexFromToBeTransformed() const {
        return indexFrom;
    }
    int translationMeasurement::getIndexToDestination() const {
        return indexTo;
    }
}

