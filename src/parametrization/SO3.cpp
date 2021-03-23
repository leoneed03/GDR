//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/SO3.h"
#include <iomanip>

namespace gdr {

//    SO3::SO3() {
//        Eigen::Matrix3d matrixId;
//        matrixId.setIdentity();
//        rotationInner = Sophus::SO3d::fitToSO3(matrixId);
//    }

//    SO3::SO3(const Sophus::SO3d &rotationSophus) {
//        rotationInner = rotationSophus;
//    }

    SO3::SO3(const Eigen::Matrix3d &rotationEigenMatrix) {
        rotationInner = Sophus::SO3d::fitToSO3(rotationEigenMatrix);
    }

    SO3::SO3(const Eigen::Quaterniond &rotationEigenQuat) {
        rotationInner = Sophus::SO3d::fitToSO3(rotationEigenQuat.normalized().toRotationMatrix());
    }


    Eigen::Vector3d SO3::getLog() const {
        return rotationInner.log();
    }


    Eigen::Quaterniond SO3::getUnitQuaternion() const {
        return rotationInner.unit_quaternion();
    }

    std::vector<double> SO3::getUnitQuaternionRawVector() const {
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

    const Sophus::SO3d &SO3::getRotationSophus() const {
        return rotationInner;
    }

    std::ostream &operator<<(std::ostream &os, const SO3 &rotation3D) {
        auto quat = rotation3D.getUnitQuaternion();
        os << "[qx, qy, qz, qw]:" << std::setw(rotation3D.getSpaceIO()) << quat.x() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.y() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.z() << ' '
        << std::setw(rotation3D.getSpaceIO()) << quat.w() << std::endl;
        return os;
    }

    SO3 operator*(const SO3& lhs, const SO3 &rhs) {
        return SO3(lhs.getRotationSophus() * rhs.getRotationSophus());
    }

    std::vector<SO3> operator*(const SO3& so3, const std::vector<SO3>& rotations) {
        std::vector<SO3> resultOrientations;

        for (const auto& rotation: rotations) {
            resultOrientations.emplace_back(so3 * rotation);
        }

        return resultOrientations;
    }

    Eigen::Matrix3d SO3::getRandomRotationMatrix3d() {

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

        rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());
        return rotationMatrix;
    }

    Eigen::Quaterniond SO3::getRandomUnitQuaternion() {

        Eigen::Matrix3d rotationMatrix = getRandomRotationMatrix3d();
        Eigen::Quaterniond quaternionRotation(rotationMatrix);

        return quaternionRotation.normalized();
    }

    SO3::SO3(const Sophus::SO3d &rotationSophus) {
        rotationInner = rotationSophus;
        assert(rotationInner.unit_quaternion().angularDistance(rotationSophus.unit_quaternion()) < std::numeric_limits<double>::epsilon());
    }

    int SO3::getSpaceIO() const {
        return spaceIomanip;
    }

    SO3 SO3::inverse() const {
        return SO3(getRotationSophus().inverse());
    }
}