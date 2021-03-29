//
// Created by leo on 3/19/21.
//

#include <fstream>

#include "readerTUM/ReaderTum.h"


namespace gdr {

    std::vector<PoseFullInfo>
    ReaderTUM::getPoseInfoTimeTranslationOrientation(const std::string &pathToGroundTruthFile) {

        std::vector<PoseFullInfo> posesInfo;
        {
            std::string currentLine;
            std::ifstream timeTranslationOrientations(pathToGroundTruthFile);
            while (std::getline(timeTranslationOrientations, currentLine)) {
                if (currentLine[0] == '#') {
                    continue;
                }
                std::stringstream poseInformation;

                double timestamp = 0.0;
                std::vector<double> translation(3, -1.0);
                std::vector<double> orientation(4, -1.0);

                poseInformation << currentLine;
                poseInformation >> timestamp;

                for (int i = 0; i < translation.size(); ++i) {
                    poseInformation >> translation[i];
                }
                for (int i = 0; i < orientation.size(); ++i) {
                    poseInformation >> orientation[i];
                }
                Eigen::Quaterniond orientationQuat(orientation.data());
                assert(orientationQuat.norm() > 0.95 && orientationQuat.norm() < 1.05);

                PoseFullInfo currentPoseInfo(timestamp,
                                             orientationQuat.normalized(),
                                             Eigen::Vector3d(translation.data()));
                posesInfo.emplace_back(currentPoseInfo);
            }
        }

        return posesInfo;
    }

}
