//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <parametrization/SE3.h>

namespace gdr {

    SE3::SE3(const Sophus::SE3d &newRelativePoseSE3): se3(newRelativePoseSE3) {}

    SE3::SE3(const Eigen::Matrix4d &poseMatrix4d) {
        se3 = Sophus::SE3d::fitToSE3(poseMatrix4d);
    }

    Sophus::SE3d SE3::getSE3Constructed() const {
        return se3;
    }

    const Sophus::SE3d &SE3::getSE3() const {
        return se3;
    }

    Eigen::Vector3d SE3::getRelativeTranslationV3() const {
        return se3.translation();
    }

    Eigen::Quaterniond SE3::getRelativeRotationSO3Quatd() const {
        return se3.unit_quaternion();
    }

    Sophus::SO3d SE3::getSO3() const {
        return se3.so3();
    }
}

