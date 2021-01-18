//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePose.h"

namespace gdr {
    relativePose::relativePose(const gdr::rotationMeasurement &newRotationMeasurement,
                               const gdr::translationMeasurement &newTranslationMeasurement) :
            relativeRotation(newRotationMeasurement),
            relativeTranslation(newTranslationMeasurement) {}

    Eigen::Quaterniond relativePose::getRotationRelative() const {
        return relativeRotation.getRotationQuat();
    }
    Eigen::Vector3d relativePose::getTranslationRelative() const {
        return relativeTranslation.getTranslation();
    }
    int relativePose::getIndexFromToBeTransformed() const {
        assert(relativeRotation.getIndexFromToBeTransformed() == relativeTranslation.getIndexFromToBeTransformed());
        return relativeRotation.getIndexFromToBeTransformed();
    }
    int relativePose::getIndexToDestination() const {
        assert(relativeRotation.getIndexToDestination() == relativeTranslation.getIndexToDestination());
        return relativeRotation.getIndexToDestination();
    }

    rotationMeasurement relativePose::getRelativeRotation() const {
        return relativeRotation;
    }

    translationMeasurement relativePose::getRelativeTranslation() const {
        return relativeTranslation;
    }
}
