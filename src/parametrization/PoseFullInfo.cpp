//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/PoseFullInfo.h"

#include <iomanip>

namespace gdr {

    PoseFullInfo::PoseFullInfo(double newTimestamp,
                               const Eigen::Quaterniond &orientationQuat,
                               const Eigen::Vector3d &coordinates) : timestamp(newTimestamp) {
        poseCameraToWorldSE3.setTranslation(coordinates);
        poseCameraToWorldSE3.setRotation(Sophus::SO3d(orientationQuat));
    }

    PoseFullInfo::PoseFullInfo(const std::vector<double> &rawPoseInfoTimestampTranslationOrientation) {
        assert(rawPoseInfoTimestampTranslationOrientation.size() == elementsRaw);
        timestamp = rawPoseInfoTimestampTranslationOrientation[0];

        int translationPosStart = 1;
        int orientationQuatPosStart = 4;

        std::vector<double> rawTranslation;
        for (int i = 0; i < 3; ++i) {
            rawTranslation.emplace_back(rawPoseInfoTimestampTranslationOrientation[translationPosStart + i]);
        }
        poseCameraToWorldSE3.setTranslation(Eigen::Map<Eigen::Vector3d>(rawTranslation.data()));

        std::vector<double> rawQuat;
        for (int i = 0; i < 4; ++i) {
            rawQuat.emplace_back(rawPoseInfoTimestampTranslationOrientation[orientationQuatPosStart + i]);
        }

        poseCameraToWorldSE3.setRotation(Sophus::SO3d(Eigen::Map<Eigen::Quaterniond>(rawQuat.data())));

    }

    std::ostream &operator<<(std::ostream &os, const PoseFullInfo &timeTranslationOrientation) {
        int space = 20;

        os.precision(std::numeric_limits<double>::max_digits10);
        os << std::setw(2 * space) << timeTranslationOrientation.timestamp << ' ';

        os.precision();
        for (int i = 0; i < 3; ++i) {
            os << timeTranslationOrientation.getTranslation()[i] << ' ';
        }

        const auto &orientationQuat = timeTranslationOrientation.poseCameraToWorldSE3.getRotationQuatd();
        os << orientationQuat.x() << ' '
           << orientationQuat.y() << ' '
           << orientationQuat.z() << ' '
           << orientationQuat.w();
        return os;
    }


    double PoseFullInfo::getTimestamp() const {
        return timestamp;
    }

    Eigen::Quaterniond PoseFullInfo::getOrientationQuat() const {
        return poseCameraToWorldSE3.getRotationQuatd();
    }

    Eigen::Vector3d PoseFullInfo::getTranslation() const {
        return poseCameraToWorldSE3.getTranslation();
    }

    Sophus::SE3d PoseFullInfo::getSophusPose() const {

        return poseCameraToWorldSE3.getSE3();
    }

    PoseFullInfo::PoseFullInfo(double newTimestamp, const SE3 &poseSE3ToSet) :
            timestamp(newTimestamp),
            poseCameraToWorldSE3(poseSE3ToSet) {}

    const SE3 &PoseFullInfo::getPoseCameraToWorldSE3() const {
        return poseCameraToWorldSE3;
    }
}
