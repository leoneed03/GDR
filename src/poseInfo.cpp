//
// Created by leoneed on 1/16/21.
//

#include "poseInfo.h"
#include <iomanip>

namespace gdr {
    poseInfo::poseInfo(double newTimestamp, const Eigen::Quaterniond &newOrientationQuat,
                       const Eigen::Vector3d &newCoordinates) : timestamp(newTimestamp),
                                                                orientationQuat(newOrientationQuat),
                                                                coordinated3d(newCoordinates) {}

    poseInfo::poseInfo(const std::vector<double> &rawPoseInfoTimestampTranslationOrientation) {
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

    std::ostream &operator<<(std::ostream &os, const poseInfo &timeTranslationOrientation) {
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


    double poseInfo::getTimestamp() const {
        return timestamp;
    }
    Eigen::Quaterniond poseInfo::getOrientationQuat() const {
        return orientationQuat;
    }

    Eigen::Vector3d poseInfo::getTranslation() const {
        return coordinated3d;
    }
}
