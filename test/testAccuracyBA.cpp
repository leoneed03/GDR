//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/AbsolutePosesComputationHandler.h"

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <thread>
#include <chrono>
#include "readerTUM/ReaderTum.h"
#include "poseGraph/ConnectedComponent.h"
#include "computationHandlers/RelativePosesComputationHandler.h"
#include "visualization/3D/SmoothPointCloud.h"

void testReconstruction(
        const std::string &shortDatasetName,
        int numberOfPosesInDataset,
        int subsamplingPeriodFrames,
        double errorTresholdR,
        double errorTresholdT,
        const gdr::CameraRGBD &cameraDefault = gdr::CameraRGBD(),
        const gdr::ParamsRANSAC &paramsRansac = gdr::ParamsRANSAC(),
        int numberOfIterations = 1,
        bool printToConsole = false,
        bool showVisualization3D = false,
        double minCoefficientOfBiggestComponent = 0.5,
        double coefficientR = 1.8,
        double coefficientT = 1.8) {

    for (int iteration = 0; iteration < numberOfIterations; ++iteration) {
        std::string numberOfPosesString = std::to_string(numberOfPosesInDataset);
        std::string frequency = std::to_string(subsamplingPeriodFrames);
        std::string datasetName = shortDatasetName + "_sampled_" + numberOfPosesString + "_" + frequency;
        std::cout << "Running test on " << datasetName << std::endl;

        std::string pathRGB = "../../data/" + datasetName + "/rgb";
        std::string pathD = "../../data/" + datasetName + "/depth";
        gdr::RelativePosesComputationHandler cgHandler(pathRGB, pathD, gdr::ParamsRANSAC(), cameraDefault);

        cgHandler.computeRelativePoses();
        cgHandler.bfsDrawToFile(
                "../../tools/data/temp/" + shortDatasetName + "_connectedComponents_" + numberOfPosesString + ".dot");
        std::vector<std::unique_ptr<gdr::AbsolutePosesComputationHandler>> connectedComponentsPoseGraph =
                cgHandler.splitGraphToConnectedComponents();

        if (printToConsole) {
            for (int componentNumber = 0; componentNumber < connectedComponentsPoseGraph.size(); ++componentNumber) {
                std::cout << " #component index by increment " << componentNumber << " of size "
                          << connectedComponentsPoseGraph[componentNumber]->getNumberOfPoses() << std::endl;
            }
        }

        auto &biggestComponent = connectedComponentsPoseGraph[0];

        std::vector<gdr::SO3> computedAbsoluteOrientationsNoRobust = biggestComponent->performRotationAveraging();
        std::vector<gdr::SO3> computedAbsoluteOrientationsRobust = biggestComponent->performRotationRobustOptimization();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = biggestComponent->performTranslationAveraging();
        std::vector<gdr::SE3> bundleAdjustedPoses = biggestComponent->performBundleAdjustmentUsingDepth();

        std::string absolutePoses = "../../data/" + datasetName + "/" + "groundtruth.txt";
        std::vector<gdr::PoseFullInfo> posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(absolutePoses);

        if (printToConsole) {
            std::cout << "read poses GT: " << posesInfoFull.size() << std::endl;
        }
        assert(posesInfoFull.size() == numberOfPosesInDataset);
        std::set<int> indicesOfBiggestComponent = biggestComponent->initialIndices();
        std::vector<gdr::PoseFullInfo> posesInfo;

        for (int poseIndex = 0; poseIndex < posesInfoFull.size(); ++poseIndex) {
            if (indicesOfBiggestComponent.find(poseIndex) != indicesOfBiggestComponent.end()) {
                posesInfo.emplace_back(posesInfoFull[poseIndex]);
            }
        }
        if (printToConsole) {
            std::cout << "sampled GT poses size: " << posesInfo.size() << std::endl;
        }
        assert(posesInfo.size() == biggestComponent->getNumberOfPoses());

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
                    "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_GT_" + numberOfPosesString +
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
                    "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_IRLS_" + numberOfPosesString +
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
                    "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_BA_" + numberOfPosesString +
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

        {
            std::cout << "__________IRLS test report " + shortDatasetName + " poses_____________" << std::endl;
            std::cout << "mean error translation: " << meanErrorT_IRLS_L2 << std::endl;
            std::cout << "mean error rotation: " << meanErrorR_IRLS_angDist << std::endl;
            std::cout << "__________BA test report " + shortDatasetName + " poses_____________" << std::endl;
            std::cout << "mean error translation: " << meanErrorT_BA_L2 << std::endl;
            std::cout << "mean error rotation: " << meanErrorR_BA_angDist << std::endl;

            std::cout << "\n poses estimated " << bundleAdjustedPoses.size() << "/" << numberOfPosesInDataset << std::endl;
        }

        if (showVisualization3D) {
            gdr::SmoothPointCloud smoothCloud;
            smoothCloud.registerPointCloudFromImage(biggestComponent->getVertices());
        }

        assert(posesGT.size() == bundleAdjustedPoses.size());
        assert(posesGT.size() >= numberOfPosesInDataset * minCoefficientOfBiggestComponent);

        ASSERT_LE(meanErrorR_BA_angDist, errorTresholdR);
        ASSERT_LE(meanErrorT_BA_L2, errorTresholdT);

        ASSERT_LE(maxErrorT_BA, maxErrorT_IRLS * coefficientT);
        ASSERT_LE(maxErrorR_BA, maxErrorR_IRLS * coefficientR);
        ASSERT_LE(meanErrorR_BA_angDist, meanErrorR_IRLS_angDist * coefficientR);
        ASSERT_LE(meanErrorT_BA_L2, meanErrorT_IRLS_L2 * coefficientT);

    }
}


TEST(testBAOptimized, visualizationDesk98) {

    testReconstruction("plant", 19, 3,
                       0.04, 0.04,
                       gdr::CameraRGBD(517.3, 318.6, 516.5, 255.3));

//    testReconstruction("desk", 98, 6,
//                       0.04, 0.04,
//                       gdr::CameraRGBD(517.3, 318.6, 516.5, 255.3));
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

