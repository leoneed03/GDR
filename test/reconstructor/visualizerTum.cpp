//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <cassert>
#include <string>

#include "boost/filesystem.hpp"

#include "readerDataset/readerTUM/ReaderTum.h"
#include "readerDataset/readerTUM/ClosestMatchFinder.h"

#include "parametrization/Reconstructable.h"
#include "visualization/3D/SmoothPointCloud.h"


namespace fs = boost::filesystem;

int main(int argc, char *argv[]) {

    std::cout << "input args format: [pathToEvaluatedTrajectory] [pathToDatasetRoot] [subsamplingPeriod] [indexPoseFixed]" << std::endl;

    assert(argc == 5);

    std::string trajectoryFile(argv[1]);
    std::string datasetRoot(argv[2]);
    int indexPoseFixed = std::stoi(std::string(argv[4]));

    const fs::path datasetPath(datasetRoot);

    fs::path assocFilePath = datasetPath;
    assocFilePath.append("assoc.txt");

    int subsamplingPeriod = std::stoi(std::string(argv[3]));

    auto posesTrajectory = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(trajectoryFile);

    assert(!posesTrajectory.empty());

    auto associatedImages = gdr::ReaderTUM::readAssocShortFilenameRgbToD(assocFilePath.string());

    std::map<double, std::pair<std::string, std::string>> rgbAndDepthNamesByDepthTime = associatedImages.getMapFromDepthTimestampToRgbAndDepthFilename();

    assert(!rgbAndDepthNamesByDepthTime.empty());

    std::vector<gdr::Reconstructable> posesToVisualize;

    std::cout << "Running visualization on " << datasetPath << " dataset" << std::endl
              << " poses file is " << trajectoryFile << " size " << posesTrajectory.size() << std::endl
              << " sampling period is " << subsamplingPeriod << std::endl
              << " index pose fixe is " << indexPoseFixed << std::endl;

    gdr::SE3 poseZero;

    if (indexPoseFixed >= 0 && indexPoseFixed < posesTrajectory.size()) {
        poseZero = posesTrajectory[indexPoseFixed].getPoseCameraToWorldSE3();
    }

    for (int poseIndex = 0; poseIndex < posesTrajectory.size(); poseIndex += subsamplingPeriod) {

        const auto &poseTrajectory = posesTrajectory[poseIndex];

        const auto &poseCameraToWorldSE3 = poseTrajectory.getPoseCameraToWorldSE3();
        double timestamp = poseTrajectory.getTimestamp();

        auto foundClosestImageInfo = gdr::findClosestKeyMatch<double, std::pair<std::string, std::string>>
                (rgbAndDepthNamesByDepthTime, timestamp);

        assert(foundClosestImageInfo != rgbAndDepthNamesByDepthTime.end());

        if (std::abs(foundClosestImageInfo->first - timestamp) > 0.02) {
            continue;
        }

        assert(std::abs(foundClosestImageInfo->first - timestamp) < 0.02);

        fs::path pathFullRgb = datasetPath;
        pathFullRgb.append("rgb");
        pathFullRgb.append(foundClosestImageInfo->second.first);

        fs::path pathFullDepth = datasetPath;
        pathFullDepth.append("depth");
        pathFullDepth.append(foundClosestImageInfo->second.second);

        gdr::CameraRGBD cameraRgbdFr1(517.3, 318.6, 516.5, 255.3);
        gdr::CameraRGBD cameraRgbdFr3(535.4, 320.1,539.2,  247.6);
        gdr::CameraRGBD cameraRgbdFr2(520.9, 325.1, 521.0, 249.7);

        const auto& cameraRgbd = cameraRgbdFr1;

        gdr::Reconstructable poseToVisualize(pathFullRgb.string(),
                                             pathFullDepth.string(),
                                             cameraRgbd);
        poseToVisualize.setAbsolutePose(poseZero.inverse() * poseCameraToWorldSE3);

        posesToVisualize.emplace_back(poseToVisualize);
    }

    gdr::SmoothPointCloud::registerPointCloudFromImages(posesToVisualize,
                                                        false,
                                                        0.005,
                                                        0.005,
                                                        0.005,
                                                        "",
                                                        "screenshot.png");

    return 0;
}
