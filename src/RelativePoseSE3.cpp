//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <RelativePoseSE3.h>

#include "RelativePoseSE3.h"

namespace gdr {

    RelativePoseSE3::RelativePoseSE3(int newIndexFrom,
                                     int newIndexTo,
                                     const Sophus::SE3d &newRelativePoseSE3):
                                     indexFrom(newIndexFrom),
                                     indexTo(newIndexTo),
                                     relativePoseSE3(newRelativePoseSE3) {}

    Sophus::SE3d RelativePoseSE3::getRelativePoseSE3Constructed() const {
        return relativePoseSE3;
    }

    int RelativePoseSE3::getIndexFrom() const {
        return indexFrom;
    }

    int RelativePoseSE3::getIndexTo() const {
        return indexTo;
    }

    const Sophus::SE3d &RelativePoseSE3::getRelativePoseSE3() const {
        return relativePoseSE3;
    }

    Eigen::Vector3d RelativePoseSE3::getRelativeTranslationV3() const {
        return relativePoseSE3.translation();
    }


    Eigen::Quaterniond RelativePoseSE3::getRelativeRotationSO3Quatd() const {
        return relativePoseSE3.unit_quaternion();
    }


    Eigen::Vector3d RelativePoseSE3::getRelativeTranslationR3() const {
        return relativePoseSE3.translation();
    }
}
