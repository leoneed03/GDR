//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <thread>
#include <chrono>
#include "poseEstimation.h"
#include "poseGraph/CorrespondenceGraph.h"
#include "readerTUM/ReaderTum.h"
#include "poseGraph/ConnectedComponent.h"
#include "computationHandlers/CorrespondenceGraphHandler.h"
#include "SmoothPointCloud.h"

#include "gnuplot_interface.h"

void plot(const std::string &func,
          double rangeFrom,
          double rangeTo) {
    {
        std::string realFuncCommand =
                "plot [" + std::to_string(rangeFrom) + ":" + std::to_string(rangeTo) + "] " + func;
        GnuplotPipe gp;
        gp.sendLine(realFuncCommand);
    }
}


TEST(testBAOptimized, visualizationDesk98) {

    for (int iterations = 0; iterations < 1; ++iterations) {
        int numberOfPosesInDataset = 98;
        double minCoefficientOfBiggestComponent = 0.4;
        std::string numberOfPosesString = std::to_string(numberOfPosesInDataset);
        double coefficientR = 1.8;
        double coefficientT = 1.8;

        std::string shortDatasetName = "desk_";
        std::string datasetName = shortDatasetName + "sampled_" + numberOfPosesString + "_6";
        std::cout << datasetName << std::endl;

        std::string pathRGB = "../../data/" + datasetName + "/rgb";
        std::string pathD = "../../data/" + datasetName + "/depth";
        gdr::CameraRGBD cameraDefault(517.3, 318.6,
                                      516.5, 255.3);
        gdr::CorrespondenceGraphHandler cgHandler(pathRGB, pathD, cameraDefault);

        const gdr::CorrespondenceGraph& correspondenceGraph = cgHandler.getCorrespondenceGraph();

        cgHandler.computeRelativePoses();
        correspondenceGraph.bfsDrawToFile(
                "../../tools/data/temp/" + shortDatasetName + "connectedComponents_" + numberOfPosesString + ".dot");
        std::vector<gdr::ConnectedComponentPoseGraph> connectedComponentsPoseGraph =
                correspondenceGraph.splitGraphToConnectedComponents();
        for (int componentNumber = 0; componentNumber < connectedComponentsPoseGraph.size(); ++componentNumber) {
            std::cout << " #component index by increment " << componentNumber << " of size "
                      << connectedComponentsPoseGraph[componentNumber].getNumberOfPoses() << std::endl;
        }
        auto &biggestComponent = connectedComponentsPoseGraph[0];

        std::vector<gdr::SO3> computedAbsoluteOrientationsNoRobust = biggestComponent.performRotationAveraging();
        std::vector<gdr::SO3> computedAbsoluteOrientationsRobust = biggestComponent.optimizeRotationsRobust();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = biggestComponent.optimizeAbsoluteTranslations();
        std::vector<gdr::SE3> bundleAdjustedPoses = biggestComponent.performBundleAdjustmentUsingDepth();

        std::string absolutePoses = "../../data/" + datasetName + "/" + "groundtruth.txt";
        std::vector<gdr::poseInfo> posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(absolutePoses);

        std::cout << "read poses GT: " << posesInfoFull.size() << std::endl;
        assert(posesInfoFull.size() == numberOfPosesInDataset);
        std::set<int> indicesOfBiggestComponent = biggestComponent.initialIndices();
        std::vector<gdr::poseInfo> posesInfo;

        for (int poseIndex = 0; poseIndex < posesInfoFull.size(); ++poseIndex) {
            if (indicesOfBiggestComponent.find(poseIndex) != indicesOfBiggestComponent.end()) {
                posesInfo.emplace_back(posesInfoFull[poseIndex]);
            }
        }
        std::cout << "sampled GT poses size: " << posesInfo.size() << std::endl;
        assert(posesInfo.size() == biggestComponent.size());

        // compute absolute poses IRLS
        std::vector<Sophus::SE3d> posesIRLS;
        {
            for (int i = 0; i < computedAbsoluteTranslationsIRLS.size(); ++i) {
                Sophus::SE3d poseIRLS;
                poseIRLS.setQuaternion(computedAbsoluteOrientationsRobust[i].getUnitQuaternion());
                poseIRLS.translation() = computedAbsoluteTranslationsIRLS[i];
                posesIRLS.push_back(poseIRLS);
            }
            // set origin at zero pose [IRLS]
            Sophus::SE3d poseIRLSzero = posesIRLS[0];
            for (auto &poseIRLS: posesIRLS) {
                poseIRLS = poseIRLSzero.inverse() * poseIRLS;
            }
        }
        {
            // set origin at zero pose [BA]
            gdr::SE3 poseBAzero = bundleAdjustedPoses[0];
            for (auto &pose: bundleAdjustedPoses) {
                pose = poseBAzero.inverse() * pose;
            }
        }

        {
            // print ground truth poses to file
            std::string outputName =
                    "../../tools/data/temp/" + shortDatasetName + "posesBiggestComponent_GT_" + numberOfPosesString +
                    ".txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[i].getSophusPose();

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }

        {
            // print poses IRLS to file
            std::string outputName =
                    "../../tools/data/temp/" + shortDatasetName + "posesBiggestComponent_IRLS_" + numberOfPosesString +
                    ".txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * posesIRLS[i];

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }
        {
            // print poses BA depth to file
            std::string outputName =
                    "../../tools/data/temp/" + shortDatasetName + "posesBiggestComponent_BA_" + numberOfPosesString +
                    ".txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * bundleAdjustedPoses[i].getSE3();

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }

        // fill absolute poses as SE3
        std::vector<Sophus::SE3d> posesGT;

        for (const auto &poseGT: posesInfo) {
            Sophus::SE3d poseSE3;
            poseSE3.setQuaternion(poseGT.getOrientationQuat());
            poseSE3.translation() = poseGT.getTranslation();
            posesGT.push_back(poseSE3);
        }
        Sophus::SE3d poseGTzero = posesGT[0];
        for (auto &poseGT: posesGT) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        double sumErrorT_BA = 0;
        double sumErrorR_BA = 0;


        double sumErrorT_IRLS = 0;
        double sumErrorR_IRLS = 0;

        double maxErrorR_IRLS = 0;
        double maxErrorT_IRLS = 0;

        double maxErrorR_BA = 0;
        double maxErrorT_BA = 0;


        for (int i = 0; i < posesGT.size(); ++i) {
            const auto &poseGT = posesGT[i];
            const auto &poseBA = bundleAdjustedPoses[i];
            const auto &poseIRLS = posesIRLS[i];

            double errorR_BA = poseGT.unit_quaternion().angularDistance(poseBA.getRotationQuatd());
            double errorT_BA = (poseGT.translation() - poseBA.getTranslation()).norm();
            sumErrorR_BA += errorR_BA;
            sumErrorT_BA += errorT_BA;

            double errorR_IRLS = poseGT.unit_quaternion().angularDistance(poseIRLS.unit_quaternion());
            double errorT_IRLS = (poseGT.translation() - poseIRLS.translation()).norm();
            sumErrorR_IRLS += errorR_IRLS;
            sumErrorT_IRLS += errorT_IRLS;

            maxErrorR_BA = std::max(errorR_BA, maxErrorR_BA);
            maxErrorT_BA = std::max(errorT_BA, maxErrorT_BA);

            maxErrorR_IRLS = std::max(errorR_IRLS, maxErrorR_IRLS);
            maxErrorT_IRLS = std::max(errorT_IRLS, maxErrorT_IRLS);
        }

        double meanErrorT_BA_L2 = sumErrorT_BA / posesGT.size();
        double meanErrorR_BA_angDist = sumErrorR_BA / posesGT.size();


        double meanErrorT_IRLS_L2 = sumErrorT_IRLS / posesGT.size();
        double meanErrorR_IRLS_angDist = sumErrorR_IRLS / posesGT.size();


        std::cout << "__________IRLS test report " + shortDatasetName + " poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_IRLS_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_IRLS_angDist << std::endl;
        std::cout << "__________BA test report " + shortDatasetName + " poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_BA_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_BA_angDist << std::endl;

        gdr::SmoothPointCloud smoothCloud;
        smoothCloud.registerPointCloudFromImage(biggestComponent.getVerticesPointers());

        assert(posesGT.size() == bundleAdjustedPoses.size());
        assert(posesGT.size() >= numberOfPosesInDataset * minCoefficientOfBiggestComponent);

        ASSERT_LE(meanErrorR_BA_angDist, 0.04); // was 0.02
        ASSERT_LE(meanErrorT_BA_L2, 0.04); // was 0.02

        ASSERT_LE(maxErrorT_BA, maxErrorT_IRLS * coefficientT);
        ASSERT_LE(maxErrorR_BA, maxErrorR_IRLS * coefficientR);
        ASSERT_LE(meanErrorR_BA_angDist, meanErrorR_IRLS_angDist * coefficientR);
        ASSERT_LE(meanErrorT_BA_L2, meanErrorT_IRLS_L2 * coefficientT);

    }
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

