//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "boost/filesystem.hpp"

#include <iostream>
#include <unordered_set>

#include "readerDataset/readerTUM/ImagesAssociator.h"
#include "readerDataset/readerTUM/ClosestMatchFinder.h"

namespace gdr {

    namespace fs = boost::filesystem;

    ImageAssociator::ImageAssociator(const std::string &pathToDirectory,
                                     const std::string &directoryNameRGB,
                                     const std::string &directoryNameD,
                                     const std::string &groundtruthFileName) :
            pathToDatasetRoot(pathToDirectory) {

        fs::path path(pathToDirectory);
        for (fs::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it) {
            std::cout << it->path().filename().string() << std::endl;
            if (it->path().filename().string() == "rgb.txt") {
                timestampByImageNameRGB = getTimestampsByImageName(it->path().string());
                for (const auto &nameAndTime: timestampByImageNameRGB) {
                    imageNamByTimestampRGB[nameAndTime.second] = nameAndTime.first;
                }
            } else if (it->path().filename().string() == "depth.txt") {
                timestampByImageNameD = getTimestampsByImageName(it->path().string());
                for (const auto &nameAndTime: timestampByImageNameD) {
                    imageNamByTimestampD[nameAndTime.second] = nameAndTime.first;
                }
            }

            if (it->path().filename().string() == directoryNameRGB) {
                fs::path rgbImages(it->path());
                for (fs::directory_iterator imagesEndIt, imagesIt(rgbImages); imagesIt != imagesEndIt; ++imagesIt) {
                    imagesPathsRGB.emplace_back(imagesIt->path().string());
                }
                std::sort(imagesPathsRGB.begin(), imagesPathsRGB.end());
            } else if (it->path().filename().string() == directoryNameD) {
                fs::path depthImages(it->path());
                for (fs::directory_iterator imagesEndIt, imagesIt(depthImages); imagesIt != imagesEndIt; ++imagesIt) {
                    imagesPathsD.emplace_back(imagesIt->path().string());
                }
                std::sort(imagesPathsD.begin(), imagesPathsD.end());
            }
        }

        std::cout << "number of rgb images from rgb.txt: " << timestampByImageNameRGB.size() << std::endl;
        std::cout << "number of depth images from depth.txt: " << timestampByImageNameD.size() << std::endl;

        int numberImagesRGB = timestampByImageNameRGB.size();
        int numberImagesD = timestampByImageNameD.size();

        assert(numberImagesRGB == imageNamByTimestampRGB.size());
        assert(numberImagesD == imageNamByTimestampD.size());
        assert(!timestampByImageNameRGB.empty() && "check that rgb.txt file is provided");
        assert(!timestampByImageNameD.empty() && "check that depth.txt file is provided");
        assert(imagesPathsRGB.size() == numberImagesRGB && "check that rgb directory is not empty");
        assert(imagesPathsD.size() == numberImagesD && "check that depth directory is not empty");
    }


    int ImageAssociator::associateImagePairs(double maxTimeTreshold,
                                             double timeOffset,
                                             const std::string &rgbDirShortName,
                                             const std::string &depthDirShortName,
                                             const std::string &groundtruthFileName,
                                             const std::string &fileExtension) {

        std::vector<std::pair<std::string, double>> shortNamesRGB;
        {
            shortNamesRGB.reserve(timestampByImageNameRGB.size());

            for (const auto &nameAndTime: timestampByImageNameRGB) {
                shortNamesRGB.emplace_back(nameAndTime);
            }
            assert(std::is_sorted(shortNamesRGB.begin(), shortNamesRGB.end()));
        }
        assert(shortNamesRGB.size() == timestampByImageNameRGB.size());

        int counter = 0;
        int notFoundCounter = 0;
        for (const auto &shortNameRGBAndTime: shortNamesRGB) {


            ++counter;
            int numberImagesRGB = timestampByImageNameRGB.size();
            int numberImagesD = timestampByImageNameD.size();

            assert(numberImagesRGB == imageNamByTimestampRGB.size());
            assert(numberImagesD == imageNamByTimestampD.size());

            double timestampToLookFor = shortNameRGBAndTime.second;
            auto closestMatchImageDepth = ClosestMatchFinder::findClosestKeyMatch<double, std::string>(
                    imageNamByTimestampD,
                    timestampToLookFor);
            auto exactMatchImageRGB = ClosestMatchFinder::findClosestKeyMatch<double, std::string>(
                    imageNamByTimestampRGB,
                    timestampToLookFor
            );
            assert(exactMatchImageRGB == imageNamByTimestampRGB.lower_bound(timestampToLookFor));

            if (closestMatchImageDepth == imageNamByTimestampD.end()) {
                ++notFoundCounter;
                continue;
            }

            if (std::abs(closestMatchImageDepth->first - timestampToLookFor) > 0.02) {
                std::cout << "STOP! " << counter << std::endl;
                ++notFoundCounter;
                continue;
            }


            double foundTimestampRGB = exactMatchImageRGB->first;
            double foundTimestampD = closestMatchImageDepth->first;
            std::string foundNameRGB = exactMatchImageRGB->second;
            std::string foundNameD = closestMatchImageDepth->second;

            associatedImagesRGBAndDepth.emplace_back(
                    std::make_pair(foundNameRGB,
                                   foundNameD));
            associatedTimestampsRGBAndDepth.emplace_back(
                    std::make_pair(foundTimestampRGB,
                                   foundTimestampD));
            assert(foundTimestampRGB == timestampToLookFor);

            imageNamByTimestampRGB.erase(exactMatchImageRGB);
            imageNamByTimestampD.erase(closestMatchImageDepth);
            timestampByImageNameRGB.erase(foundNameRGB);
            timestampByImageNameD.erase(foundNameD);

            // pair found successfully

            std::cout.precision(std::numeric_limits<double>::max_digits10);
            std::cout << "looking for " << timestampToLookFor << " (time) "
                      << shortNameRGBAndTime.first << std::endl;
        }

        std::cout << "not found: " << notFoundCounter << " of " << counter << std::endl;

        double maxDiff = -1;

        assert(associatedImagesRGBAndDepth.size() == associatedTimestampsRGBAndDepth.size());
        for (int i = 0; i < associatedImagesRGBAndDepth.size(); ++i) {
            std::cout << "      paired: \t" << associatedImagesRGBAndDepth[i].first
                      << " == \t" << associatedImagesRGBAndDepth[i].second << "\n time: "
                      << " \t" << associatedTimestampsRGBAndDepth[i].first << " & \t"
                      << associatedTimestampsRGBAndDepth[i].second << "\n";
            double timeDiff = std::abs(
                    associatedTimestampsRGBAndDepth[i].first - associatedTimestampsRGBAndDepth[i].second);
            assert(timeDiff < maxTimeTreshold);
            maxDiff = std::max(maxDiff, timeDiff);

        }
        std::cout << "paired successfully " << associatedTimestampsRGBAndDepth.size() << " of " << counter << std::endl;
        std::cout << " max diff is " << maxDiff << std::endl;

        fs::path pathToGT = fs::path(pathToDatasetRoot);
        pathToGT.append(groundtruthFileName);

        std::vector<double> timestampsOfChosenRGB;
        timestampsOfChosenRGB.reserve(associatedTimestampsRGBAndDepth.size());
        for (const auto &timestampsPair: associatedTimestampsRGBAndDepth) {
            timestampsOfChosenRGB.emplace_back(timestampsPair.first);
        }

        std::unordered_set<double> foundTimestampsRGB;
        associatedGroundTruthInfo = getAssociatedPoseInfo(pathToGT.string(),
                                                          timestampsOfChosenRGB,
                                                          foundTimestampsRGB);

        std::cout << "end..." << std::endl;

        std::vector<std::pair<std::string, std::string>> newAssociatedImageNamesRGBAndD;
        std::vector<std::pair<double, double>> newAssociatedTimestampsRGBAndD;

        assert(!foundTimestampsRGB.empty());
        for (int i = 0; i < associatedTimestampsRGBAndDepth.size(); ++i) {
            double timestampRGB = associatedTimestampsRGBAndDepth[i].first;

            if (foundTimestampsRGB.find(timestampRGB) == foundTimestampsRGB.end()) {
                continue;
            }

            newAssociatedTimestampsRGBAndD.emplace_back(associatedTimestampsRGBAndDepth[i]);
            newAssociatedImageNamesRGBAndD.emplace_back(associatedImagesRGBAndDepth[i]);
        }

        std::swap(associatedTimestampsRGBAndDepth, newAssociatedTimestampsRGBAndD);
        std::swap(associatedImagesRGBAndDepth, newAssociatedImageNamesRGBAndD);
        assert(associatedGroundTruthInfo.size() == associatedImagesRGBAndDepth.size());
        assert(associatedImagesRGBAndDepth.size() == associatedTimestampsRGBAndDepth.size());

        for (int i = 0; i < associatedTimestampsRGBAndDepth.size(); ++i) {
            double timeDiff = associatedTimestampsRGBAndDepth[i].first - associatedGroundTruthInfo[i].getTimestamp();
            assert(std::abs(timeDiff) < maxTimeTreshold);
        }

        std::cout << "this is max error between new GT timestamps and RGB timestamps" << std::endl;
        std::cout << "totally matched pairs: " << associatedGroundTruthInfo.size() << std::endl;
        return 0;
    }

    std::map<std::string, double> ImageAssociator::getTimestampsByImageName(const std::string &pathToTimestampsFile) {
        std::map<std::string, double> timestampByNameToReturn;

        {
            std::string currentLine;
            std::ifstream timestampsByImageNameStream(pathToTimestampsFile);
            while (std::getline(timestampsByImageNameStream, currentLine)) {
                if (currentLine[0] == '#') {
                    continue;
                }

                std::stringstream poseInformation;
                poseInformation << currentLine;
                double timestamp = 0;
                poseInformation >> timestamp;
                std::string imageNameShort;
                poseInformation >> imageNameShort;

                timestampByNameToReturn[imageNameShort] = timestamp;

            }
        }

        return timestampByNameToReturn;
    }

    std::vector<PoseFullInfo>
    ImageAssociator::getAssociatedPoseInfo(const std::string &pathToGT,
                                           const std::vector<double> &timestamps,
                                           std::unordered_set<double> &putFoundTimestamps,
                                           double maxTimeTreshold,
                                           double timeOffset) {
        std::vector<PoseFullInfo> absolutePosesFoundByTimestamps;
        std::map<double, PoseFullInfo> poseInfoByTimestamps;
        std::unordered_map<double, int> stampIndexByTime;
        std::unordered_set<double> foundTimestamps;
        foundTimestamps.reserve(timestamps.size());
        stampIndexByTime.reserve(timestamps.size());

        for (int i = 0; i < timestamps.size(); ++i) {
            stampIndexByTime.insert(std::make_pair(timestamps[i], i));
        }

        assert(stampIndexByTime.size() == timestamps.size());

        int notFoundCounter = 0;

        {
            std::string currentLine;
            std::ifstream timestampsByImageNameStream(pathToGT);
            while (std::getline(timestampsByImageNameStream, currentLine)) {
                if (currentLine[0] == '#') {
                    continue;
                }

                std::stringstream poseInformation;

                double timestamp = 0;
                std::vector<double> translation(3, -1);
                std::vector<double> orientation(4, -1);

                poseInformation << currentLine;
                poseInformation >> timestamp;

                for (int i = 0; i < translation.size(); ++i) {
                    poseInformation >> translation[i];
                }
                for (int i = 0; i < orientation.size(); ++i) {
                    poseInformation >> orientation[i];
                }

                PoseFullInfo currentPoseInfo(timestamp,
                                             Eigen::Quaterniond(orientation.data()),
                                             Eigen::Vector3d(translation.data()));
                poseInfoByTimestamps.insert(std::make_pair(timestamp, currentPoseInfo));

            }
        }

        for (const auto &timestamp: timestamps) {
            auto closestMatch = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(poseInfoByTimestamps,
                                                                                              timestamp);
            if (closestMatch == poseInfoByTimestamps.end()) {
                std::cout << " did not find timestamp [FOUND END ?!] " << timestamp << std::endl;
                ++notFoundCounter;
                continue;
            }
            if (std::abs(closestMatch->first - timestamp) > maxTimeTreshold) {
                std::cout << " did not find timestamp " << timestamp
                          << " closest is " << closestMatch->first << std::endl;
                ++notFoundCounter;
                continue;
            }
            assert(std::abs(timestamp - closestMatch->first) < maxTimeTreshold);
            assert(std::abs(timestamp - closestMatch->second.getTimestamp()) < maxTimeTreshold);
            foundTimestamps.insert(timestamp);
            absolutePosesFoundByTimestamps.emplace_back(closestMatch->second);
        }

        std::cout << " GT found poses size is " << absolutePosesFoundByTimestamps.size()
                  << " timestamps size is " << timestamps.size() << std::endl;
        std::cout << "did not find matches for " << notFoundCounter << std::endl;

        double maxTimeDiff = -1;

        int indexInAbsolutePoses = 0;
        for (int i = 0; i < timestamps.size(); ++i) {

            double timestampToFind = timestamps[i];
            if (foundTimestamps.find(timestamps[i]) == foundTimestamps.end()) {
                continue;
            }

            double timestampGT = absolutePosesFoundByTimestamps[indexInAbsolutePoses].getTimestamp();
            double timeDiff = std::abs(timestampGT - timestampToFind);
            maxTimeDiff = std::max(maxTimeDiff, timeDiff);
            assert(timeDiff < maxTimeTreshold);
            ++indexInAbsolutePoses;
        }
        std::cout << "Should be almost zero " << maxTimeDiff << std::endl;
        assert(maxTimeDiff < maxTimeTreshold);
        std::swap(foundTimestamps, putFoundTimestamps);
        return absolutePosesFoundByTimestamps;
    }

    int ImageAssociator::exportAllInfoToDirectory(const std::string &outDirectory,
                                                  const std::set<int> &indicesToSampleConst,
                                                  bool exportGT,
                                                  const std::string &shortNameRGBDir,
                                                  const std::string &shortNameDepthDir,
                                                  const std::string &shortFilenameGT,
                                                  const std::string &extension) {

        associateImagePairs();
        const fs::path path(outDirectory);
        fs::create_directory(path);

        auto pathRGB = path;
        pathRGB.append(shortNameRGBDir);
        fs::create_directory(pathRGB);


        auto pathD = path;
        pathD.append(shortNameDepthDir);
        fs::create_directory(pathD);

        auto indicesToSample = indicesToSampleConst;

        if (indicesToSample.empty()) {
            for (int i = 0; i < checkSize(); ++i) {
                indicesToSample.insert(i);
            }
        }
        checkSize();
        const fs::path outDirRoot(outDirectory);
        fs::path outRGB = outDirRoot;
        outRGB.append(shortNameRGBDir);


        fs::path outD = outDirRoot;
        outD.append(shortNameDepthDir);

        for (int i = 0; i < associatedImagesRGBAndDepth.size(); ++i) {

            if (indicesToSample.find(i) == indicesToSample.end()) {
                continue;
            }

            const auto &imagesRGBAndDToBeExported = associatedImagesRGBAndDepth[i];
            fs::path imageRGB(pathToDatasetRoot);
            imageRGB.append(imagesRGBAndDToBeExported.first);
            fs::path imageRGBToSave = outRGB;
            imageRGBToSave.append(imageRGB.filename().string());
            fs::copy_file(imageRGB, imageRGBToSave);


            fs::path imageD(pathToDatasetRoot);
            imageD.append(imagesRGBAndDToBeExported.second);
            fs::path imageDToSave = outD;
            imageDToSave.append(imageD.filename().string());
            fs::copy_file(imageD, imageDToSave);
        }

        fs::path outGT = outDirRoot;
        std::string filenameGT = shortFilenameGT + "." + extension;
        outGT.append(filenameGT);


        std::ofstream groundtruthFile(outGT.string());

        groundtruthFile << "# ground truth trajectory" << std::endl;
        groundtruthFile << "# file: datasetname" << std::endl;
        groundtruthFile << "# timestamp tx ty tz qx qy qz qw" << std::endl;

        for (int indexGT = 0; indexGT < associatedGroundTruthInfo.size(); ++indexGT) {

            if (indicesToSample.find(indexGT) == indicesToSample.end()) {
                continue;
            }

            const auto &absolutePose = associatedGroundTruthInfo[indexGT];
            groundtruthFile.precision(std::numeric_limits<double>::max_digits10);
            groundtruthFile << absolutePose << std::endl;
        }

        return 0;
    }

    int ImageAssociator::checkSize() const {

        assert(associatedImagesRGBAndDepth.size() == associatedTimestampsRGBAndDepth.size());
        assert(associatedGroundTruthInfo.size() == associatedImagesRGBAndDepth.size());

        return associatedGroundTruthInfo.size();
    }

    std::vector<PoseFullInfo> ImageAssociator::getGroundtruthForGivenTimestamps(const std::vector<double> &timestamps,
                                                                                const std::vector<PoseFullInfo> &posesGT,
                                                                                double maxTimeDiff) {

        assert(std::is_sorted(timestamps.begin(), timestamps.end()));
        std::vector<PoseFullInfo> posesToReturn;
        std::map<double, PoseFullInfo> setOfPosesGT;

        for (const auto &poseGT: posesGT) {
            setOfPosesGT.insert(std::make_pair(poseGT.getTimestamp(), poseGT));
        }

        assert(!setOfPosesGT.empty());

        for (const auto &timestamp: timestamps) {
            auto foundPoseGT = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(
                    setOfPosesGT, timestamp);

            assert(foundPoseGT != setOfPosesGT.end());
            assert(std::abs(timestamp - foundPoseGT->first) < maxTimeDiff
                   && "no ground truth pose was found for this timestamp");

            posesToReturn.emplace_back(foundPoseGT->second);
        }

        assert(posesToReturn.size() == timestamps.size());

        return posesToReturn;
    }
}