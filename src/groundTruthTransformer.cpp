//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#define spaceIO (15)

#include "groundTruthTransformer.h"
#include "printer.h"

#include <vector>
#include <limits>

#include <boost/filesystem.hpp>
#include <Eigen/LU>
#include <iomanip>
#include <set>


namespace gdr {


    namespace fs = boost::filesystem;

    void putAligned(std::ofstream &of, const std::vector<double> &val) {

        for (const auto &vals: val) {
            (of << std::setw(12) << vals << ' ');
        }
    }

    int
    GTT::makeRotationsRelative(const std::string &pathToGroundTruth,
                               const std::string &pathToRelativeGroundTruth) {

        std::ifstream in(pathToGroundTruth);
        std::ofstream out(pathToRelativeGroundTruth);
        int numOfEmptyLines = 3;
        int numbersInLine = 8;
        std::vector<double> stamp0;
        std::vector<double> prevCoordinates = {0, 0, 0};
        bool isZero = true;

        if (in) {
            for (int i = 0; i < numOfEmptyLines; ++i) {
                std::string s;
                std::getline(in, s);
            }
            double currVal = -1;

            while (true) {
                std::vector<double> stamps;
                for (int i = 0; i < numbersInLine; ++i) {
                    if (in >> currVal) {
                        stamps.push_back(currVal);
                    } else {
                        return 1;
                    }
                }
                assert(stamps.size() == numbersInLine);
                if (isZero) {
                    stamp0 = stamps;
                    isZero = false;
                }

                std::vector<double> vectorData0 = {stamp0[4], stamp0[5], stamp0[6], stamp0[7]};
                std::vector<double> vectorData = {stamps[4], stamps[5], stamps[6], stamps[7]};
                Eigen::Quaterniond qd(vectorData.data());
                Eigen::Quaterniond q0(vectorData0.data());
                auto qRelative = q0.inverse() * qd;
                std::vector<double> toStream = {stamps[0] - stamp0[0], stamps[1] - prevCoordinates[0],
                                                stamps[2] - prevCoordinates[1], stamps[3] - prevCoordinates[2],
                                                qRelative.x(), qRelative.y(), qRelative.z(), qRelative.w()};
                putAligned(out, toStream);
                out << std::endl;
                prevCoordinates = {stamps[1], stamps[2], stamps[3]};
            }
        }
        return 0;
    }

    std::vector<std::string> readData(std::string pathToRGB) {

        DIR *pDIR;
        struct dirent *entry;
        std::vector<std::string> RgbImages;
        PRINT_PROGRESS("start reading");
        if ((pDIR = opendir(pathToRGB.data())) != nullptr) {
            while ((entry = readdir(pDIR)) != nullptr) {
                if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
                    continue;
                }
                std::string rPathToRgbImage = entry->d_name;
                RgbImages.emplace_back(rPathToRgbImage);
            }
            closedir(pDIR);
            std::sort(RgbImages.begin(), RgbImages.end());
        }
        for (int i = 0; i < RgbImages.size(); ++i) {
            PRINT_PROGRESS(i + 1 << "::" << RgbImages[i]);
        }
        return RgbImages;
    }

    std::pair<std::vector<std::string>, std::vector<std::string>>
    GTT::makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB,
                                               const std::string &pathToD, const std::string &pathOutDirectory,
                                               const std::string &timeInfo,
                                               const std::set<int> &indices) {

        std::ifstream in(pathToGroundTruth);
        std::string outRGB = pathOutDirectory + "/rgb";
        std::string outD = pathOutDirectory + "/depth";

        namespace fs = boost::filesystem;
        fs::path path_to_remove(pathOutDirectory);
        for (fs::directory_iterator end_dir_it, it(path_to_remove); it != end_dir_it; ++it) {
            fs::remove_all(it->path());
        }

        boost::filesystem::create_directory(outD);
        boost::filesystem::create_directory(outRGB);
        std::vector<double> stamp0;
        std::vector<double> prevCoordinates = {0, 0, 0};
        std::vector<std::string> rgbDataR = readData(pathToRGB);
        std::vector<std::string> dDataR = readData(pathToD);
        std::vector<std::string> rgbData = readRgbData(pathToRGB);
        std::vector<std::string> dData = readRgbData(pathToD);

        assert(rgbData.size() == dData.size());
        assert(!rgbData.empty());

        std::vector<std::string> onlyTakenRGB;
        int cntr = 0;
        for (const auto &e: indices) {
            if (e >= rgbData.size()) {
                break;
            }
            std::string toRGB = outRGB + "/" + rgbDataR[e];
            std::string toD = outD + "/" + dDataR[e];
            PRINT_PROGRESS("write RGB " << toRGB);
            PRINT_PROGRESS("write D " << toD);
            boost::filesystem::copy_file(rgbData[e], toRGB);
            boost::filesystem::copy_file(dData[e], toD);
            std::cout << e << " copied rgb " << toRGB << std::endl;
            PRINT_PROGRESS("success" << std::endl);
            onlyTakenRGB.push_back(rgbDataR[e]);
            ++cntr;
        }

        PRINT_PROGRESS("pathOut" << pathOutDirectory);
        writeInfo(onlyTakenRGB, timeInfo, pathToGroundTruth, pathOutDirectory + "/groundtruth_new.txt",
                  pathOutDirectory + "/relative_groundtruth.txt", indices);
        return {rgbDataR, dDataR};
    }

    std::vector<double> GTT::createTimestamps(const std::vector<std::string> &rgb,
                                              const std::string &pathTimeRGB, const std::string &pathToGroundTruth,
                                              const std::set<int> &indices) {

        std::map<std::string, double> rgbToTime;
        int skipNum = 3;
        std::vector<double> timeStamps;
        std::ifstream in(pathTimeRGB);
        std::vector<double> stamp0;
        std::vector<double> prevCoordinates = {0, 0, 0};
        int index = 0;

        if (in) {
            for (int i = 0; i < skipNum; ++i) {
                std::string s;
                std::getline(in, s);
            }
            double currVal = -1;
            std::string currPath;

            while (true) {
                std::vector<double> stamps;
                if (timeStamps.size() == rgb.size()) {
                    return timeStamps;
                }
                if (in >> currVal && in >> currPath) {
                    if (currPath == ("rgb/" + rgb[index])) {
                        rgbToTime[rgb[index]] = currVal;
                        timeStamps.push_back(currVal);
                        ++index;
                    }
                } else {

                    if (timeStamps.size() == rgb.size()) {
                        return timeStamps;
                    } else {
                        PRINT_PROGRESS(timeStamps.size() << " vs " << rgb.size());
                        PRINT_PROGRESS(timeStamps[timeStamps.size() - 1] << " vs " << rgb[rgb.size() - 1]);
                        return timeStamps;
                    }
                }
                assert(index <= rgb.size());
            }
        }
        return timeStamps;
    }

    std::vector<std::vector<double>>
    GTT::getGroundTruth(const std::string &pathToGroundTruth, const std::vector<double> &timeStamps) {

        std::ifstream in(pathToGroundTruth);
        int numOfEmptyLines = 3;
        int numbersInLine = 8;
        std::vector<double> stamp0;
        std::vector<double> prevCoordinates = {0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<std::vector<double>> coordAndQuat;

        if (in) {
            for (int i = 0; i < numOfEmptyLines; ++i) {
                std::string s;
                std::getline(in, s);
            }
            double currVal = -1;
            bool run = true;
            while (run) {
                std::vector<double> stamps;
                for (int i = 0; i < numbersInLine; ++i) {
                    if (in >> currVal) {
                        stamps.push_back(currVal);
                    } else {
                        run = false;
                        break;
                    }
                }
                if (run) {
                    assert(stamps.size() == 8);
                    coordAndQuat.push_back(stamps);
                }
            }
        }
        std::vector<std::vector<double>> resultingTruth;
        for (int i = 0; i < timeStamps.size(); ++i) {
            for (int posInFile = 0; posInFile < coordAndQuat.size(); ++posInFile) {
                if (abs(coordAndQuat[posInFile][0] - timeStamps[i]) < abs(prevCoordinates[0] - timeStamps[i])) {
                    prevCoordinates = coordAndQuat[posInFile];
                }
            }
            resultingTruth.push_back(prevCoordinates);
        }
        assert(resultingTruth.size() == timeStamps.size());

        return resultingTruth;
    }

    int
    GTT::writeGroundTruth(const std::string &pathOut, const std::vector<std::vector<double>> &timeCoordinates) {

        std::ofstream out(pathOut);
        int skipN = 3;
        for (int i = 0; i < skipN; ++i) {
            out << "#\n";
        }

        for (const auto &e: timeCoordinates) {
            out.precision(std::numeric_limits<double>::max_digits10);
            out << e[0];
//            out << std::setw(2 * spaceIO) << e[0];

            for (int i = 1; i < e.size(); ++i) {

                out << ' ' << e[i];
//                out << std::setw(2 * spaceIO) << e[i];
            }
            out << std::endl;
        }
        return 0;
    }

    int GTT::writeGroundTruthRelativeToZeroPose(const std::string &pathOut,
                                                const std::vector<std::vector<double>> &timeCoordinates) {

        std::ofstream out(pathOut);
        int skipN = 3;
        for (int i = 0; i < skipN; ++i) {
            out << "#\n";
        }

        Eigen::Matrix3d zeroRotationMatrix;
        Eigen::Vector3d zeroTranslation;

        for (int index = 0; index < timeCoordinates.size(); ++index) {
            auto &e = timeCoordinates[index];

            out.precision(std::numeric_limits<double>::max_digits10);
            out << std::setw(2 * spaceIO) << e[0];
            Eigen::Vector3d currentTranslation;
            std::vector<double> vectorData = {e[4], e[5], e[6], e[7]};
            Eigen::Quaterniond qd(vectorData.data());
            Eigen::Matrix3d currentRotationMatrix = qd.toRotationMatrix();

            currentTranslation[0] = e[1];
            currentTranslation[1] = e[2];
            currentTranslation[2] = e[3];

            if (index == 0) {
                zeroRotationMatrix = currentRotationMatrix;
                zeroTranslation = currentTranslation;
            }

            Eigen::Matrix3d matrixDouble = zeroRotationMatrix.transpose() * currentRotationMatrix;
            Eigen::Quaterniond qRelatived(matrixDouble);

            Eigen::Vector3d deltaT = currentTranslation - zeroTranslation;
            Eigen::Vector3d relativeTranslation = zeroRotationMatrix.inverse() * deltaT;
            for (int posTranslation = 0; posTranslation < 3; ++posTranslation) {
                out << std::setw(2 * spaceIO) << relativeTranslation.col(0)[posTranslation];
            }
            out << std::setw(2 * spaceIO) << qRelatived.x() << std::setw(2 * spaceIO) << qRelatived.y()
                << std::setw(2 * spaceIO) << qRelatived.z() << std::setw(2 * spaceIO) << qRelatived.w() << std::endl;
        }
        return 0;
    }

    void GTT::writeInfo(const std::vector<std::string> &rgb, const std::string &pathTimeRGB,
                        const std::string &pathToGroundTruth, const std::string &pathOut,
                        const std::string &relativeOutput,
                        const std::set<int> &indices) {
        std::vector<double> timeStamps = createTimestamps(rgb, pathTimeRGB, pathToGroundTruth, indices);
        std::vector<std::vector<double>> timeAndCoordinates = getGroundTruth(pathToGroundTruth, timeStamps);
        writeGroundTruth(pathOut, timeAndCoordinates);
        writeGroundTruthRelativeToZeroPose(relativeOutput, timeAndCoordinates);
    }

    void
    GTT::prepareDataset(const std::string &pathToDataset,
                        const std::string &pathOut,
                        const std::set<int> &indicesSet,
                        const std::string &NewName = "subset") {

        std::string pathNewOut = pathOut + "/" + NewName;
        std::string groundtruth = pathToDataset + "/groundtruth.txt";
        std::string rgb = pathToDataset + "/rgb";
        std::string depth = pathToDataset + "/depth";
        std::string timeInfo = pathToDataset + "/rgb.txt";
        makeRotationsRelativeAndExtractImages(groundtruth,
                                              rgb,
                                              depth,
                                              pathNewOut,
                                              timeInfo,
                                              indicesSet);
    }

    std::vector<std::vector<double>> GTT::extractTimeAndTransformation(const std::string &inputFileName) {

        std::ifstream in(inputFileName);

        int lineSize = 8;
        std::vector<std::vector<double>> timeAndTransformation;

        if (in) {
            std::string s;
            for (int i = 0; i < 3; ++i) {
                std::getline(in, s);
                if (s.empty()) {
                    return timeAndTransformation;
                }
            }

            int counterLines = 0;
            double currentValue;
            std::vector<double> currentPoseInfo;
            while (in >> currentValue) {
                ++counterLines;
                currentPoseInfo.push_back(currentValue);

                if (counterLines == lineSize) {
                    counterLines = 0;
                    timeAndTransformation.push_back(currentPoseInfo);
                    currentPoseInfo.clear();
                }

            }
        }
        return timeAndTransformation;
    }

    std::vector<relativePose>
    GTT::extractAllRelativeTransformationPairwise(const std::string &in, const std::string &pathOut,
                                                  std::string noise) {

        std::vector<relativePose> relativePoses;
        std::vector<std::vector<double>> timeAndAbsolutePoses = extractTimeAndTransformation(in);
        std::ofstream out(pathOut);

        for (int index = 0; index < timeAndAbsolutePoses.size(); ++index) {
            int elementsPoseInfo = 8;
            auto &pose = timeAndAbsolutePoses[index];
            assert(pose.size() == elementsPoseInfo);
            std::vector<double> zeroPose(elementsPoseInfo - 1, 0);
            zeroPose.push_back(1);
            out << "VERTEX_SE3:QUAT " << std::setw(spaceIO) << index;
            //we can actually explicitly set orientation of the zero pose but is it necessary?
            for (int i = 1; i < zeroPose.size(); ++i) {
                out << std::setw(spaceIO) << zeroPose[i];//timeAndAbsolutePoses[0][i];//pose[i];
            }
            out << std::endl;
        }

        for (int index = 0; index < timeAndAbsolutePoses.size(); ++index) {

            std::vector<double> &currentPose = timeAndAbsolutePoses[index];
            std::vector<double> rotationFrom = {currentPose[4], currentPose[5], currentPose[6], currentPose[7]};
            std::vector<double> translationFrom = {currentPose[1], currentPose[2], currentPose[3]};
            Eigen::Quaterniond qFrom(rotationFrom.data());
            qFrom.normalize();
            Eigen::Vector3d tFrom(translationFrom.data());
            Sophus::SE3d fromSE3;
            fromSE3.setQuaternion(qFrom);
            fromSE3.translation() = tFrom;
            fromSE3 = fromSE3.inverse();

            qFrom = fromSE3.unit_quaternion();
            tFrom = fromSE3.translation();

            for (int to = index + 1; to < timeAndAbsolutePoses.size(); ++to) {

                double maxDiff = 1e-11;
                std::vector<double> &currentPoseTo = timeAndAbsolutePoses[to];
                std::vector<double> rotationTo = {currentPoseTo[4], currentPoseTo[5], currentPoseTo[6],
                                                  currentPoseTo[7]};
                std::vector<double> translationTo = {currentPoseTo[1], currentPoseTo[2], currentPoseTo[3]};
                Eigen::Quaterniond qTo(rotationTo.data());
                qTo.normalize();
                Eigen::Vector3d tTo(translationTo.data());

                Sophus::SE3d toSE3;
                toSE3.setQuaternion(qTo);
                toSE3.translation() = tTo;
                toSE3 = toSE3.inverse();

                qTo = toSE3.unit_quaternion();
                tTo = toSE3.translation();

                //// R_{ij} = R_{j}^T * R_{i}
                Eigen::Quaterniond relativeRotationQuat = qTo.inverse() * qFrom;
                relativeRotationQuat.normalize();
                double angularDistanceComputationTypeMatrixOrQuaternion = Eigen::Quaterniond(
                        qTo.inverse().toRotationMatrix() * qFrom.toRotationMatrix()).normalized().angularDistance(
                        relativeRotationQuat);

                assert(angularDistanceComputationTypeMatrixOrQuaternion < maxDiff);
                //// t_{ij} = R_{j}^T * (t_i - t_j)
                Eigen::Vector3d relativeTranslation = qTo.inverse().toRotationMatrix() * (tFrom - tTo);

                ////Alternative way of relative pose computation:
                Eigen::Matrix4d poseFrom;
                poseFrom.setIdentity();
                poseFrom.block<3, 3>(0, 0) = qFrom.toRotationMatrix();
                poseFrom.block<3, 1>(0, 3) = tFrom;

                Eigen::Matrix4d poseTo;
                poseTo.setIdentity();
                poseTo.block<3, 3>(0, 0) = qTo.toRotationMatrix();
                poseTo.block<3, 1>(0, 3) = tTo;

                Eigen::Matrix4d relativePoseMatrix4d = (poseTo.inverse() * poseFrom);
                Eigen::Vector3d relativeTranslationFromFullPose = relativePoseMatrix4d.block<3, 1>(0, 3);
                Eigen::Matrix3d relativeRotationFromFullPose = relativePoseMatrix4d.block<3, 3>(0, 0);
                double angularDistanceRotations = relativeRotationQuat.angularDistance(
                        Eigen::Quaterniond(relativeRotationFromFullPose));
                Eigen::Vector3d translationDifferenceShouldBeZero =
                        relativeTranslationFromFullPose - relativeTranslation;

                ///Check that alternative way returns same result for relative poses
                assert(angularDistanceRotations < maxDiff);
                bool translationDiffCloseToZero = translationDifferenceShouldBeZero.norm() < maxDiff;
                if (!translationDiffCloseToZero) {
                    std::cout << "translation diff is " << translationDifferenceShouldBeZero.norm() << std::endl;
                }
                assert(translationDiffCloseToZero);

                //// index->to == i->j
                out << "EDGE_SE3:QUAT " << std::setw(spaceIO) << to << std::setw(spaceIO) << index;

                rotationMeasurement relativeRotationMeasurement(relativeRotationQuat, index, to);
                translationMeasurement relativeTranslationMeasurement(relativeTranslation, index, to);
                relativePoses.push_back(relativePose(relativeRotationMeasurement, relativeTranslationMeasurement));
                for (int posTranslation = 0; posTranslation < 3; ++posTranslation) {
                    out << std::setw(spaceIO) << relativeTranslation.col(0)[posTranslation];
                }
                out << std::setw(spaceIO) << relativeRotationQuat.x() << std::setw(spaceIO) << relativeRotationQuat.y()
                    << std::setw(spaceIO) << relativeRotationQuat.z() << std::setw(spaceIO) << relativeRotationQuat.w()
                    << noise << std::endl;
            }
        }
        return relativePoses;
    }

    std::vector<relativePose> GTT::extractAllRelativeTransformationPairwise(const std::string &in) {

        std::vector<relativePose> relativePoses;
        std::vector<std::vector<double>> timeAndAbsolutePoses = extractTimeAndTransformation(in);

        for (int index = 0; index < timeAndAbsolutePoses.size(); ++index) {
            int elementsPoseInfo = 8;
            auto &pose = timeAndAbsolutePoses[index];
            assert(pose.size() == elementsPoseInfo);
            std::vector<double> zeroPose(elementsPoseInfo - 1, 0);
            zeroPose.push_back(1);
        }

        for (int index = 0; index < timeAndAbsolutePoses.size(); ++index) {

            std::vector<double> &currentPose = timeAndAbsolutePoses[index];
            std::vector<double> rotationFrom = {currentPose[4], currentPose[5], currentPose[6], currentPose[7]};
            std::vector<double> translationFrom = {currentPose[1], currentPose[2], currentPose[3]};
            Eigen::Quaterniond qFrom(rotationFrom.data());
            qFrom.normalize();
            Eigen::Vector3d tFrom(translationFrom.data());

            for (int to = index + 1; to < timeAndAbsolutePoses.size(); ++to) {

                double maxDiff = 1e-11;
                std::vector<double> &currentPoseTo = timeAndAbsolutePoses[to];
                std::vector<double> rotationTo = {currentPoseTo[4], currentPoseTo[5], currentPoseTo[6],
                                                  currentPoseTo[7]};
                std::vector<double> translationTo = {currentPoseTo[1], currentPoseTo[2], currentPoseTo[3]};
                Eigen::Quaterniond qTo(rotationTo.data());
                qTo.normalize();
                Eigen::Vector3d tTo(translationTo.data());

                //// R_{ij} = R_{j}^T * R_{i}
                Eigen::Quaterniond relativeRotationQuat = qTo.inverse() * qFrom;
                relativeRotationQuat.normalize();
                double angularDistanceComputationTypeMatrixOrQuaternion = Eigen::Quaterniond(
                        qTo.inverse().toRotationMatrix() * qFrom.toRotationMatrix()).normalized().angularDistance(
                        relativeRotationQuat);

                assert(angularDistanceComputationTypeMatrixOrQuaternion < maxDiff);
                //// t_{ij} = R_{j}^T * (t_i - t_j)
                Eigen::Vector3d relativeTranslation = qTo.inverse().toRotationMatrix() * (tFrom - tTo);

                ////Alternative way of relative pose computation:
                Eigen::Matrix4d poseFrom;
                poseFrom.setIdentity();
                poseFrom.block<3, 3>(0, 0) = qFrom.toRotationMatrix();
                poseFrom.block<3, 1>(0, 3) = tFrom;

                Eigen::Matrix4d poseTo;
                poseTo.setIdentity();
                poseTo.block<3, 3>(0, 0) = qTo.toRotationMatrix();
                poseTo.block<3, 1>(0, 3) = tTo;

                Eigen::Matrix4d relativePoseMatrix4d = (poseTo.inverse() * poseFrom);
                Eigen::Vector3d relativeTranslationFromFullPose = relativePoseMatrix4d.block<3, 1>(0, 3);
                Eigen::Matrix3d relativeRotationFromFullPose = relativePoseMatrix4d.block<3, 3>(0, 0);
                double angularDistanceRotations = relativeRotationQuat.angularDistance(
                        Eigen::Quaterniond(relativeRotationFromFullPose));
                Eigen::Vector3d translationDifferenceShouldBeZero =
                        relativeTranslationFromFullPose - relativeTranslation;

                ///Check that alternative way returns same result for relative poses
                assert(angularDistanceRotations < maxDiff);
                bool translationDiffCloseToZero = translationDifferenceShouldBeZero.norm() < maxDiff;
                if (!translationDiffCloseToZero) {
                    std::cout << "translation diff is " << translationDifferenceShouldBeZero.norm() << std::endl;
                }
                assert(translationDiffCloseToZero);

                rotationMeasurement relativeRotationMeasurement(relativeRotationQuat, index, to);
                translationMeasurement relativeTranslationMeasurement(relativeTranslation, index, to);
                relativePoses.push_back(relativePose(relativeRotationMeasurement, relativeTranslationMeasurement));
            }
        }
        return relativePoses;
    }

    std::vector<poseInfo> GTT::getPoseInfoTimeTranslationOrientation(const std::string &pathToGroundTruthFile) {
        std::vector<poseInfo> posesInfo;
        std::vector<std::vector<double>> rawPoseInfo = extractTimeAndTransformation(pathToGroundTruthFile);

        std::cout << "========================================================" << std::endl;
        for (const auto &line: rawPoseInfo) {
            for (const auto &element: line) {
                std::cout << element << ' ';
            }
            std::cout << std::endl;
        }
        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++=+++" << std::endl;
        for (const auto &rawPose: rawPoseInfo) {
            assert(rawPose.size() == infoElements);

            poseInfo currentPose(rawPose);
            posesInfo.emplace_back(currentPose);
        }
        return posesInfo;
    }


    int GTT::printRelativePosesFile(const std::vector<relativePose> &relativePoses,
                                    const std::string &pathOutRelativePoseFile, int numPoses) {

        std::ofstream file(pathOutRelativePoseFile);

        if (file.is_open()) {

            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }
            for (const auto &relPose: relativePoses) {
                {
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = relPose.getIndexToDestination();
                    int indexFrom = relPose.getIndexFromToBeTransformed();
                    assert(indexTo > indexFrom);
                    assert(indexTo < numPoses);
                    assert(indexFrom < numPoses);

                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)(gtsam format)
                    file << "EDGE_SE3:QUAT " << indexTo << ' ' << indexFrom << ' ';
                    auto translationVector = relPose.getTranslationRelative();
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &qR = relPose.getRotationRelative();

                    std::vector<double> vectorDataRotations = {qR.x(), qR.y(), qR.z(), qR.w()};
                    file << std::to_string(qR.x()) << ' ' << std::to_string(qR.y()) << ' ' <<
                         std::to_string(qR.z()) << ' '
                         << std::to_string(qR.w()) << noise << '\n';
                }
            }
        }

        return 0;
    }

    std::vector<relativePose> GTT::readRelativePoses(const std::string &relativePosesFile) {

        std::vector<relativePose> resultRelativePoses;

        std::ifstream in(relativePosesFile);
        int numbersInLine = 8;
        int qDim = 4;
        int dim = 3;
        if (in) {
            while (true) {
                std::string currentToken = "_";
                while (currentToken[0] != 'E') {
                    if (!(in >> currentToken)) {
                        return resultRelativePoses;
                    }
                }
                int indexFrom = -1;
                int indexTo = -1;
                in >> indexTo;
                in >> indexFrom;
                assert(indexFrom < indexTo);


                std::vector<double> currentT(3, 0);
                for (int i = 0; i < dim; ++i) {
                    in >> currentT[i];
                }

                std::vector<double> currentPose(4, 0);
                for (int i = 0; i < qDim; ++i) {
                    in >> currentPose[i];
                }
                resultRelativePoses.push_back(
                        relativePose(rotationMeasurement(Eigen::Quaterniond(currentPose.data()), indexFrom, indexTo),
                                     translationMeasurement(Eigen::Vector3d(currentT.data()), indexFrom, indexTo)));

            }
        }


        return resultRelativePoses;
//        std::ifstream in(pathToGroundTruth);
//        int numOfEmptyLines = 3;
//        int numbersInLine = 8;
//        std::vector<double> stamp0;
//        std::vector<double> prevCoordinates = {0, 0, 0, 0, 0, 0, 0, 0};
//        std::vector<std::vector<double>> coordAndQuat;
//
//        if (in) {
//            for (int i = 0; i < numOfEmptyLines; ++i) {
//                std::string s;
//                std::getline(in, s);
//            }
//            double currVal = -1;
//            bool run = true;
//            while (run) {
//                std::vector<double> stamps;
//                for (int i = 0; i < numbersInLine; ++i) {
//                    if (in >> currVal) {
//                        stamps.push_back(currVal);
//                    } else {
//                        run = false;
//                        break;
//                    }
//                }
//                if (run) {
//                    assert(stamps.size() == 8);
//                    coordAndQuat.push_back(stamps);
//                }
//            }
//        }
//        std::vector<std::vector<double>> resultingTruth;
//        for (int i = 0; i < timeStamps.size(); ++i) {
//            for (int posInFile = 0; posInFile < coordAndQuat.size(); ++posInFile) {
//                if (abs(coordAndQuat[posInFile][0] - timeStamps[i]) < abs(prevCoordinates[0] - timeStamps[i])) {
//                    prevCoordinates = coordAndQuat[posInFile];
//                }
//            }
//            resultingTruth.push_back(prevCoordinates);
//        }
//        assert(resultingTruth.size() == timeStamps.size());
//
//        return resultingTruth;
    }

    GTT::EXIT_CODES_CLASS
    GTT::createDepthTxtFile(const std::string &pathToRgbdDirectory, const std::string &toBeCreatedFilename) {

        fs::path p(pathToRgbdDirectory);

        std::cout << "start reading depth directory " << std::endl;
        std::vector<std::pair<std::string, std::string>> pairOfTimestampAndDotExtension;

        if (exists(p)) {
            if (is_regular_file(p)) {
                std::cout << "provide a directory path as argument, not a regular file" << std::endl;
                return EXIT_CODES_CLASS::NOT_A_DIRECTORY_ERROR;
            } else if (is_directory(p)) {
                for (const auto &file: fs::directory_iterator(p)) {
                    std::cout << file.path() << std::endl;
                    fs::path depthFile = file.path();

                    for (const auto &namePart: depthFile) {
                        if (namePart.has_extension()) {
                            std::cout << namePart.stem() << " vs " << namePart.extension() << std::endl;
                            pairOfTimestampAndDotExtension.push_back(
                                    {namePart.stem().string(), namePart.extension().string()});
                        }
                    }
                }
            } else {
                std::cout << p << "provide a directory path as argument" << std::endl;
                return EXIT_CODES_CLASS::NOT_A_DIRECTORY_ERROR;
            }
        } else {
            std::cout << p << "invalid path" << std::endl;
            return EXIT_CODES_CLASS::INVALID_DIRECTORY_PATH;
        }

        std::sort(pairOfTimestampAndDotExtension.begin(), pairOfTimestampAndDotExtension.end());
        std::ofstream depthTxtFile(toBeCreatedFilename);

        depthTxtFile << "# depth maps" << std::endl;
        depthTxtFile << "# file: rgbd_dataset_freiburg1_plant_19_3.bag" << std::endl;
        depthTxtFile << "# timestamp filename" << std::endl;
        for (const auto &timestampAndExtension: pairOfTimestampAndDotExtension) {
            depthTxtFile << timestampAndExtension.first << ' ' << "depth/" << timestampAndExtension.first + timestampAndExtension.second << std::endl;
        }


        return EXIT_CODES_CLASS::OK;
    }

}