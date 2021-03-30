//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/SO3.h"

#include <iomanip>

namespace gdr {

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

    const Sophus::SO3d &SO3::getRotationSophus() const {
        return rotationInner;
    }

    std::ostream &operator<<(std::ostream &os, const SO3 &rotation3D) {
        const auto &quat = rotation3D.getUnitQuaternion();
        os << quat.x() << ' '
           << quat.y() << ' '
           << quat.z() << ' '
           << quat.w();

        return os;
    }

    SO3 operator*(const SO3 &lhs, const SO3 &rhs) {
        return SO3(lhs.getRotationSophus() * rhs.getRotationSophus());
    }

    std::vector<SO3> operator*(const SO3 &so3, const std::vector<SO3> &rotations) {
        std::vector<SO3> resultOrientations;

        for (const auto &rotation: rotations) {
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
        assert(rotationInner.unit_quaternion().angularDistance(rotationSophus.unit_quaternion()) <
               std::numeric_limits<double>::epsilon());
    }

    int SO3::getSpaceIO() const {
        return spaceIomanip;
    }

    SO3 SO3::inverse() const {
        return SO3(getRotationSophus().inverse());
    }
}