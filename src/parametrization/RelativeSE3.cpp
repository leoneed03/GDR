//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/RelativeSE3.h"


namespace gdr {

    int RelativeSE3::getIndexTo() const {
        return indexToToBeTransformed;
    }

    int RelativeSE3::getIndexFrom() const {
        return indexFromDestination;
    }

    SO3 RelativeSE3::getRelativeRotation() const {
        return relativePose.getSO3();
    }

    Eigen::Vector3d RelativeSE3::getRelativeTranslation() const {
        return relativePose.getTranslation();
    }

    Sophus::SE3d RelativeSE3::getRelativePoseSE3() const {
        return relativePose.getSE3();
    }

    const SE3& RelativeSE3::getRelativePose() const {
        return relativePose;
    }

    RelativeSE3::RelativeSE3(int indexFromDestinationToSet,
                             int indexToToBeTransformedToSet,
                             const SE3 &se3):
            indexFromDestination(indexFromDestinationToSet),
            indexToToBeTransformed(indexToToBeTransformedToSet),
            relativePose(se3) {}
}
