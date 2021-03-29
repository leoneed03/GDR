//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/translationAveraging/TranslationMeasurement.h"

namespace gdr {

    TranslationMeasurement::TranslationMeasurement(const Eigen::Vector3d &newTranslation3d, int newIndexFrom,
                                                   int newIndexTo) : translation3d(newTranslation3d),
                                                                     indexFrom(newIndexFrom),
                                                                     indexTo(newIndexTo) {}
    const Eigen::Vector3d& TranslationMeasurement::getTranslation() const {
        return translation3d;
    }


    int TranslationMeasurement::getIndexFromToBeTransformed() const {
        return indexFrom;
    }
    int TranslationMeasurement::getIndexToDestination() const {
        return indexTo;
    }
}

