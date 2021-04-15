//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <fstream>
#include <set>

#include "readerDataset/readerTUM/ReaderTum.h"
#include "readerDataset/readerTUM/ClosestMatchFinder.h"

#include "boost/filesystem.hpp"

namespace gdr {

    namespace fs = boost::filesystem;


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

    AssociatedImages ReaderTUM::readAssocShortFilenameRgbToD(const std::string &pathToAssocFile) {
        AssociatedImages associatedImages;
        std::ifstream assocFile(pathToAssocFile);

        assert(assocFile.is_open());

        std::string currentLine;
        while (std::getline(assocFile, currentLine)) {

            if (currentLine.empty()) {
                continue;
            }
            if (currentLine[0] == '#') {
                continue;
            }
            std::stringstream associatedInfoTimeRgbTimeD;

            double timeRgb = 0.0;
            double timeD = 0.0;

            std::string rgbFile;
            std::string dFile;

            associatedInfoTimeRgbTimeD << currentLine;


            associatedInfoTimeRgbTimeD >> timeRgb;
            associatedInfoTimeRgbTimeD >> rgbFile;
            associatedInfoTimeRgbTimeD >> timeD;
            associatedInfoTimeRgbTimeD >> dFile;

            fs::path rgbLongName(rgbFile);
            fs::path dLongName(dFile);

            std::string rgbShortName = rgbLongName.filename().string();
            std::string dShortName = dLongName.filename().string();

            associatedImages.timeAndPairedDepthByRgb[rgbShortName] = {timeRgb, dShortName};
            associatedImages.timeAndPairedRgbByDepth[dShortName] = {timeD, rgbShortName};

        }

        assert(associatedImages.timeAndPairedDepthByRgb.size() == associatedImages.timeAndPairedRgbByDepth.size());
        assert(!associatedImages.timeAndPairedDepthByRgb.empty());

        return associatedImages;
    }

    std::vector<PoseFullInfo>
    ReaderTUM::getPoseInfoTimeTranslationOrientationByMatches(const std::vector<PoseFullInfo> &posesFullInfo,
                                                              const std::vector<double> &timestampsToSearchFor,
                                                              double timeDifTreshold) {

        std::vector<PoseFullInfo> matchingGroundTruthPoses;
        std::vector<double> timestamps(timestampsToSearchFor);

        std::sort(timestamps.begin(), timestamps.end());

        std::map<double, PoseFullInfo> timestampsSearchable;

        for (const auto &poseFullInfo: posesFullInfo) {

            double timestampOfPoseGt = poseFullInfo.getTimestamp();

            if (timestampsSearchable.find(timestampOfPoseGt) != timestampsSearchable.end()) {
                timestampsSearchable.insert(std::make_pair(timestampOfPoseGt, poseFullInfo));
            } else {
                timestampsSearchable.insert(std::make_pair(timestampOfPoseGt, poseFullInfo));;
            }
        }

        //all timestamps should be unique
        assert(timestampsSearchable.size() == posesFullInfo.size());
        assert(!timestampsSearchable.empty() && "check that groundtruth poses file is not empty");

        for (const auto &timestampToFind: timestamps) {
            auto iterFound = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(timestampsSearchable,
                                                                                           timestampToFind);
            assert(iterFound != timestampsSearchable.end());

            if (std::abs(iterFound->first - timestampToFind) < timeDifTreshold) {
                matchingGroundTruthPoses.emplace_back(iterFound->second);
            }
        }

        return matchingGroundTruthPoses;
    }

    void ReaderTUM::createRgbOrDepthTxt(const std::string &pathToImagesRgb,
                                        const std::string &pathToCreatedFile) {
        const fs::path imageDir(pathToImagesRgb);

        std::string imageType = imageDir.filename().string();
        fs::path toSave(pathToCreatedFile);

        assert(imageType == "depth" || imageType == "rgb");

        toSave.append(imageType + ".txt");
        std::ofstream outputFile(toSave.string());

        std::set<std::string> timestamps;
        std::set<std::string> names;

        for (fs::directory_iterator end_dir_it, it(imageDir); it != end_dir_it; ++it) {
            auto filename = it->path().filename().string();

            names.insert(filename);
        }

        for (const auto& name: names) {

            fs::path path(imageType);
            path.append(name);

            auto timeString = name.substr(0, name.length() - 4);
            outputFile << timeString << ' ' << path.string() << std::endl;
        }
    }

}
