//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/translationAveraging/RelativePoseMeasurement.h"

namespace gdr {

    RelativePoseMeasurement::RelativePoseMeasurement(const gdr::RotationMeasurement &newRotationMeasurement,
                                                     const gdr::TranslationMeasurement &newTranslationMeasurement) :
            relativeRotation(newRotationMeasurement),
            relativeTranslation(newTranslationMeasurement) {}

    Eigen::Quaterniond RelativePoseMeasurement::getRotationRelative() const {
        return relativeRotation.getRotationQuat();
    }

    Eigen::Vector3d RelativePoseMeasurement::getTranslationRelative() const {
        return relativeTranslation.getTranslation();
    }

    int RelativePoseMeasurement::getIndexFromToBeTransformed() const {
        assert(relativeRotation.getIndexFromToBeTransformed() == relativeTranslation.getIndexFromToBeTransformed());
        return relativeRotation.getIndexFromToBeTransformed();
    }

    int RelativePoseMeasurement::getIndexToDestination() const {
        assert(relativeRotation.getIndexToDestination() == relativeTranslation.getIndexToDestination());
        return relativeRotation.getIndexToDestination();
    }
}
