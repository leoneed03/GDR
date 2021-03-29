//
// Created by leoneed on 1/16/21.
//

#include "readerTUM/PoseFullInfo.h"
#include <iomanip>

namespace gdr {
    PoseFullInfo::PoseFullInfo(double newTimestamp, const Eigen::Quaterniond &newOrientationQuat,
                               const Eigen::Vector3d &newCoordinates) : timestamp(newTimestamp),
                                                                orientationQuat(newOrientationQuat),
                                                                coordinated3d(newCoordinates) {}

    PoseFullInfo::PoseFullInfo(const std::vector<double> &rawPoseInfoTimestampTranslationOrientation) {
        assert(rawPoseInfoTimestampTranslationOrientation.size() == elementsRaw);
        timestamp = rawPoseInfoTimestampTranslationOrientation[0];

        int translationPosStart = 1;
        int orientationQuatPosStart = 4;

        for (int i = 0; i < 3; ++i) {
            coordinated3d[i] = rawPoseInfoTimestampTranslationOrientation[translationPosStart + i];
        }

        std::vector<double> rawQuat;
        for (int i = 0; i < 4; ++i) {
            rawQuat.push_back(rawPoseInfoTimestampTranslationOrientation[orientationQuatPosStart + i]);
        }

        orientationQuat = Eigen::Quaterniond(rawQuat.data());


    }

    std::ostream &operator<<(std::ostream &os, const PoseFullInfo &timeTranslationOrientation) {
        int space = 15;

        os.precision(std::numeric_limits<double>::max_digits10);
        os << std::setw(2 * space) << timeTranslationOrientation.timestamp;
        os.precision(space - 1);
        for (int i = 0; i < timeTranslationOrientation.coordinated3d.size(); ++i) {
            os << std::setw(space) << timeTranslationOrientation.coordinated3d[i];
        }

        os << std::setw(space) << timeTranslationOrientation.orientationQuat.x()
           << std::setw(space) << timeTranslationOrientation.orientationQuat.y()
           << std::setw(space) << timeTranslationOrientation.orientationQuat.z()
           << std::setw(space) << timeTranslationOrientation.orientationQuat.w();
        return os;
    }


    double PoseFullInfo::getTimestamp() const {
        return timestamp;
    }
    Eigen::Quaterniond PoseFullInfo::getOrientationQuat() const {
        return orientationQuat;
    }

    Eigen::Vector3d PoseFullInfo::getTranslation() const {
        return coordinated3d;
    }

    Sophus::SE3d PoseFullInfo::getSophusPose() const {
        Sophus::SE3d poseSE3;
        poseSE3.setQuaternion(orientationQuat);
        poseSE3.translation() = coordinated3d;

        return poseSE3;
    }
}
