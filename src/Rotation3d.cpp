//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "Rotation3d.h"

namespace gdr {

    Rotation3d::Rotation3d() {
        Eigen::Matrix3d matrixId;
        matrixId.setIdentity();
        rotationInner = Sophus::SO3d::fitToSO3(matrixId);
    }

//    Rotation3d::Rotation3d(const Sophus::SO3d &rotationSophus) {
//        rotationInner = rotationSophus;
//    }

    Rotation3d::Rotation3d(const Eigen::Matrix3d &rotationEigenMatrix) {
        rotationInner = Sophus::SO3d::fitToSO3(rotationEigenMatrix);
    }

    Rotation3d::Rotation3d(const Eigen::Quaterniond &rotationEigenQuat) {
        rotationInner = Sophus::SO3d::fitToSO3(rotationEigenQuat.normalized().toRotationMatrix());
    }


    Eigen::Vector3d Rotation3d::getLog() const {
        return rotationInner.log();
    }


    Eigen::Quaterniond Rotation3d::getUnitQuaternion() const {
        return rotationInner.unit_quaternion();
    }


    Rotation3d& Rotation3d::operator=(const Rotation3d& right) {

        if (this == &right) {
            return *this;
        }
        rotationInner = right.getRotationSophus();
        return *this;
    }

    const Sophus::SO3d &Rotation3d::getRotationSophus() const {
        return rotationInner;
    }
    std::ostream& operator<<(std::ostream& os, const Rotation3d& rotation3D) {
//        os << rotation3D.rotationInner;
        os << rotation3D.getRotationSophus().matrix();
        return os;
    }
}