//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "Rotation3d.h"
#include <iomanip>

namespace gdr {

//    Rotation3d::Rotation3d() {
//        Eigen::Matrix3d matrixId;
//        matrixId.setIdentity();
//        rotationInner = Sophus::SO3d::fitToSO3(matrixId);
//    }

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

    std::vector<double> Rotation3d::getUnitQuaternionRawVector() const {
        Eigen::Quaterniond quat = rotationInner.unit_quaternion();
        quat.normalize();

        int dimQuat = 4;
        std::vector<double> rawQuat;
        rawQuat.reserve(dimQuat);
        rawQuat.push_back(quat.x());
        rawQuat.push_back(quat.y());
        rawQuat.push_back(quat.z());
        rawQuat.push_back(quat.w());

        return rawQuat;
    }

    const Sophus::SO3d &Rotation3d::getRotationSophus() const {
        return rotationInner;
    }

    std::ostream &operator<<(std::ostream &os, const Rotation3d &rotation3D) {
        auto quat = rotation3D.getUnitQuaternion();
        os << "[qx, qy, qz, qw]:" << std::setw(rotation3D.getSpaceIO()) << quat.x() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.y() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.z() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.w() << std::endl;
        return os;
    }

    Eigen::Matrix3d Rotation3d::getRandomRotationMatrix3d() {


        int dim = 3;
        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        double maxRotation = 30;

        std::uniform_real_distribution<> distrib(0, maxRotation);
        Eigen::Matrix3d rotationMatrix;

        std::vector<double> angles;
        for (int i = 0; i < dim; ++i) {
            angles.push_back(distrib(randomNumberGenerator));
        }

        rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());
        return rotationMatrix;
    }

    Eigen::Quaterniond Rotation3d::getRandomUnitQuaternion() {

        Eigen::Matrix3d rotationMatrix = getRandomRotationMatrix3d();
        Eigen::Quaterniond quaternionRotation(rotationMatrix);
        quaternionRotation.normalize();

        return quaternionRotation;
    }

    Rotation3d::Rotation3d(const Sophus::SO3d &rotationSophus) {
        rotationInner = rotationSophus;
        assert(rotationInner.unit_quaternion().angularDistance(rotationSophus.unit_quaternion()) < std::numeric_limits<double>::epsilon());
    }

    int Rotation3d::getSpaceIO() const {
        return spaceIOiomanip;
    }
}