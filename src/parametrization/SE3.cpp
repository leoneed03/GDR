//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <parametrization/SE3.h>

namespace gdr {

    SE3::SE3(const Sophus::SE3d &newRelativePoseSE3) :
            se3(newRelativePoseSE3) {}

    SE3::SE3(const Eigen::Matrix4d &poseMatrix4d) {
        se3 = Sophus::SE3d::fitToSE3(poseMatrix4d);
    }

    SE3::SE3(const Eigen::Quaterniond &rotation,
             const Eigen::Vector3d &translation) {

        se3.translation() = translation;
        se3.setQuaternion(rotation);
    }

    void SE3::setRotation(const Sophus::SO3d &rotationToSet) {
        se3.setQuaternion(rotationToSet.unit_quaternion());
    }

    void SE3::setRotation(const SO3 &rotationToSet) {
        se3.setQuaternion(rotationToSet.getUnitQuaternion());
    }

    void SE3::setTranslation(const Eigen::Vector3d &translationToSet) {
        se3.translation() = translationToSet;
    }

    Sophus::SE3d SE3::getSE3Constructed() const {
        return se3;
    }

    const Sophus::SE3d &SE3::getSE3() const {
        return se3;
    }

    Eigen::Vector3d SE3::getTranslation() const {
        return se3.translation();
    }

    Eigen::Quaterniond SE3::getRotationQuatd() const {
        return se3.unit_quaternion();
    }

    Sophus::SO3d SE3::getSO3() const {
        return se3.so3();
    }

    SE3 SE3::inverse() const {
        return SE3(se3.inverse());
    }

    SE3 operator*(const SE3 &lhs, const SE3 &rhs) {
        return SE3(lhs.getSE3() * rhs.getSE3());
    }

    std::pair<double, double> SE3::getRotationTranslationErrors(const SE3 &otherSe3) {

        double errorL2 = 0;
        double errorAngle = 0;

        errorAngle = getRotationQuatd().angularDistance(otherSe3.getRotationQuatd());
        errorL2 = (getTranslation() - otherSe3.getTranslation()).norm();

        return {errorAngle, errorL2};
    }


    std::ostream &operator<<(std::ostream &os, const SE3 &pose) {
        const auto &translation = pose.getTranslation();

        for (int i = 0; i < 3; ++i) {
            os << translation[i] << ' ';
        }

        os << SO3(pose.getSO3());

        return os;
    }

    SE3 SE3::getRandomSE3(double maxTranslation) {

        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());

        std::uniform_real_distribution<> distrib(-maxTranslation, maxTranslation);

        Eigen::Vector3d translation;

        assert(translation.size() == 3);

        for (int i = 0; i < translation.size(); ++i) {
            translation[i] = distrib(randomNumberGenerator);
        }

        Eigen::Quaterniond rotation = SO3::getRandomUnitQuaternion();

        return SE3(rotation, translation);
    }
}

