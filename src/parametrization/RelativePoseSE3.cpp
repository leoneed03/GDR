//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <parametrization/RelativePoseSE3.h>

#include "parametrization/RelativePoseSE3.h"

namespace gdr {

    RelativePoseSE3::RelativePoseSE3(int newIndexFrom,
                                     int newIndexTo,
                                     const SE3 &newRelativePoseSE3):

                                     indexFrom(newIndexFrom),
                                     indexTo(newIndexTo),
                                     relativePoseSE3(newRelativePoseSE3) {}

    SE3 RelativePoseSE3::getRelativePoseSE3Constructed() const {
        return relativePoseSE3;
    }

    int RelativePoseSE3::getIndexFrom() const {
        return indexFrom;
    }

    int RelativePoseSE3::getIndexTo() const {
        return indexTo;
    }

    const SE3 &RelativePoseSE3::getRelativePoseSE3() const {
        return relativePoseSE3;
    }

    Eigen::Vector3d RelativePoseSE3::getTranslation() const {
        return relativePoseSE3.getTranslation();
    }


    Eigen::Quaterniond RelativePoseSE3::getRelativeRotationSO3Quatd() const {
        return relativePoseSE3.getRotationQuatd();
    }

    Sophus::SO3d RelativePoseSE3::getRotation() const {
        return relativePoseSE3.getSO3();
    }

}
