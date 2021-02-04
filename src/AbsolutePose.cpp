//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "AbsolutePose.h"

namespace gdr {
    AbsolutePose::AbsolutePose(const Sophus::SE3d &poseSE3) {
        absolutePoseSE3 = poseSE3;
    }

    AbsolutePose::AbsolutePose(const Eigen::Matrix4d &poseMatrix4d) {
        absolutePoseSE3 = Sophus::SE3d::fitToSE3(poseMatrix4d);
    }

    AbsolutePose::AbsolutePose(const Eigen::Quaterniond &rotationQuatd, const Eigen::Vector3d &translationVector3d) {
        absolutePoseSE3.setQuaternion(rotationQuatd);
        absolutePoseSE3.translation() = translationVector3d;

        assert((absolutePoseSE3.translation() - translationVector3d).norm() < std::numeric_limits<double>::epsilon());
    }


    Sophus::SE3d AbsolutePose::getSophusSE3d() const {
        return absolutePoseSE3;
    }

    Sophus::SO3d AbsolutePose::getRotationSO3() const {
        return absolutePoseSE3.so3();
    }

    Eigen::Vector3d AbsolutePose::getTranslationVector3d() const {
        return absolutePoseSE3.translation();
    }

    Rotation3d AbsolutePose::getRotation3d() const {
        return Rotation3d(absolutePoseSE3.so3());
    }
}
