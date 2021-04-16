//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/AbsolutePosesComputationHandler.h"

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <chrono>

#include "readerDataset/readerTUM/ReaderTum.h"

#include "computationHandlers/RelativePosesComputationHandler.h"
#include "computationHandlers/ModelCreationHandler.h"

#include "readerDataset/readerTUM/Evaluator.h"

void testReconstruction(
        const std::string &shortDatasetName,
        int numberOfPosesInDataset,
        int subsamplingPeriodFrames,
        double errorTresholdR,
        double errorTresholdT,
        const gdr::CameraRGBD &cameraDefault = gdr::CameraRGBD(),
        const gdr::ParamsRANSAC &paramsRansac = gdr::ParamsRANSAC(),
        const std::string &assocFile = "",
        int numberOfIterations = 1,
        bool printToConsole = false,
        bool showVisualization3D = false,
        double minCoefficientOfBiggestComponent = 0.5,
        double coefficientR = 1.2,
        double coefficientT = 1.2,
        double timeDiffThreshold = 0.02) {

    for (int iteration = 0; iteration < numberOfIterations; ++iteration) {
        std::string numberOfPosesString = std::to_string(numberOfPosesInDataset);
        std::string frequency = std::to_string(subsamplingPeriodFrames);
        std::string datasetName = shortDatasetName;
        std::cout << "Running test on " << datasetName << std::endl;
        std::string pathRelativeToData = "../../data/";
        std::string pathRGB = pathRelativeToData + datasetName + "/rgb";
        std::string pathD = pathRelativeToData + datasetName + "/depth";

        std::string pathAssoc = (assocFile != "") ? (pathRelativeToData + datasetName + "/" + assocFile) : (assocFile);

        gdr::RelativePosesComputationHandler cgHandler(pathRGB,
                                                       pathD,
                                                       gdr::DatasetDescriber(cameraDefault, pathAssoc),
                                                       paramsRansac);


        std::cout << "start computing relative poses" << std::endl;
        cgHandler.computeRelativePoses();

        cgHandler.bfsDrawToFile(
                "../../tools/data/temp/" + shortDatasetName + "_connectedComponents_" + numberOfPosesString + ".dot");
        std::vector<std::unique_ptr<gdr::AbsolutePosesComputationHandler>> connectedComponentsPoseGraph =
                cgHandler.splitGraphToConnectedComponents();

        std::cout << "Biggest component of size "
                  << connectedComponentsPoseGraph[0]->getNumberOfPoses() << std::endl;
        if (printToConsole) {
            for (int componentNumber = 0; componentNumber < connectedComponentsPoseGraph.size(); ++componentNumber) {
                std::cout << " #component index by increment " << componentNumber << " of size "
                          << connectedComponentsPoseGraph[componentNumber]->getNumberOfPoses() << std::endl;
            }
        }

        auto &biggestComponent = connectedComponentsPoseGraph[0];

        std::cout << "perform rotation averaging" << std::endl;
        std::vector<gdr::SO3> computedAbsoluteOrientationsNoRobust = biggestComponent->performRotationAveraging();

        std::cout << "perform rotation robust optimization" << std::endl;
        std::vector<gdr::SO3> computedAbsoluteOrientationsRobust = biggestComponent->performRotationRobustOptimization();

        std::cout << "perform translation averaging" << std::endl;
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = biggestComponent->performTranslationAveraging();
        std::vector<gdr::SE3> irlsPoses = biggestComponent->getPosesSE3();





        std::vector<gdr::SE3> bundleAdjustedPoses;
        std::cout << "perform Bundle Adjustment" << std::endl;
        bundleAdjustedPoses = biggestComponent->performBundleAdjustmentUsingDepth();

        std::string absolutePosesGroundTruth = "../../data/" + datasetName + "/" + "groundtruth.txt";
        std::vector<gdr::PoseFullInfo> posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesGroundTruth);

        std::vector<double> timestampsToFind = biggestComponent->getPosesTimestamps();

        std::vector<gdr::PoseFullInfo> posesFullInfoIRLS;

        for (int i = 0; i < irlsPoses.size(); ++i) {
            const auto &pose = irlsPoses[i];
            posesFullInfoIRLS.emplace_back(gdr::PoseFullInfo(timestampsToFind[i], pose));
        }

        std::vector<gdr::PoseFullInfo> posesFullInfoBA;

        for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
            const auto &pose = bundleAdjustedPoses[i];
            posesFullInfoBA.emplace_back(gdr::PoseFullInfo(timestampsToFind[i], pose));
        }

        assert(posesFullInfoBA.size() == posesFullInfoIRLS.size());
        assert(!posesFullInfoIRLS.empty());
        assert(posesFullInfoIRLS.size() == timestampsToFind.size());

        posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientationByMatches(posesInfoFull,
                                                                                       timestampsToFind,
                                                                                       timeDiffThreshold);

        if (true) {

            assert(!posesInfoFull.empty());
            std::set<int> indicesOfBiggestComponent = biggestComponent->initialIndices();

            std::vector<gdr::PoseFullInfo> posesInfo;

            for (int poseIndex = 0; poseIndex < posesInfoFull.size(); ++poseIndex) {
                //TODO
//            if (indicesOfBiggestComponent.find(poseIndex) != indicesOfBiggestComponent.end()) {
                posesInfo.emplace_back(posesInfoFull[poseIndex]);
//            }
            }
            if (printToConsole) {
                std::cout << "sampled GT poses size: " << posesInfo.size() << std::endl;
            }
//        assert(posesInfo.size() == biggestComponent->getNumberOfPoses());

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
                        "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_GT_" +
                        numberOfPosesString +
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
                        "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_IRLS_" +
                        numberOfPosesString +
                        ".txt";
                std::ofstream computedPoses(outputName);
                for (int i = 0; i < posesInfo.size(); ++i) {
                    Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * posesIRLS[0].inverse() * posesIRLS[i];

                    computedPoses.precision(std::numeric_limits<double>::max_digits10);
                    computedPoses << timestampsToFind[i] << ' ';
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
                        "../../tools/data/temp/" + shortDatasetName + "_posesBiggestComponent_BA_" +
                        numberOfPosesString +
                        ".txt";
                std::ofstream computedPoses(outputName);
                for (int i = 0; i < posesInfo.size(); ++i) {
                    Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() *
                                           bundleAdjustedPoses[0].getSE3().inverse() * bundleAdjustedPoses[i].getSE3();

                    computedPoses.precision(std::numeric_limits<double>::max_digits10);
                    computedPoses << timestampsToFind[i] << ' ';
                    const auto to = poseSE3.translation();
                    for (int j = 0; j < 3; ++j) {
                        computedPoses << to[j] << ' ';
                    }
                    auto quatComputed = poseSE3.unit_quaternion();

                    computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                                  << quatComputed.w() << std::endl;
                }
            }

            gdr::Evaluator evaluator(absolutePosesGroundTruth);

            double meanErrorRotBA = 0;
            double meanErrorL2BA = 0;

            double meanErrorRotIRLS = 0;
            double meanErrorL2IRLS = 0;

            {

                {

                    assert(!posesFullInfoBA.empty());
                    auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoBA,
                                                                          biggestComponent->getIndexFixedPose());
                    std::cout << "========================BA report:=========================" << std::endl;
                    std::cout << informationErrors.rotationError << "------------------------------------" << std::endl;
                    std::cout << informationErrors.translationError << std::endl;

                    meanErrorRotBA = informationErrors.rotationError.MEAN;
                    meanErrorL2BA = informationErrors.translationError.MEAN;
                }
                {

                    assert(!posesFullInfoIRLS.empty());
                    auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoIRLS,
                                                                          biggestComponent->getIndexFixedPose());

                    std::cout << "========================IRLS report:=========================" << std::endl;
                    std::cout << informationErrors.rotationError << "------------------------------------" << std::endl;
                    std::cout << informationErrors.translationError << std::endl;

                    meanErrorL2IRLS = informationErrors.translationError.MEAN;
                    meanErrorRotIRLS = informationErrors.rotationError.MEAN;
                }

            }

            if (showVisualization3D) {
                gdr::ModelCreationHandler modelCreationHandler(biggestComponent->getPoseGraph());
                modelCreationHandler.visualize();
//                modelCreationHandler.saveAsPly("test.ply");
            }

            assert(bundleAdjustedPoses.size() >= numberOfPosesInDataset * minCoefficientOfBiggestComponent);

            ASSERT_LE(meanErrorRotBA, errorTresholdR);
            ASSERT_LE(meanErrorL2BA, errorTresholdT);

            ASSERT_LE(meanErrorRotBA, meanErrorRotIRLS * coefficientR);
            ASSERT_LE(meanErrorL2BA, meanErrorL2IRLS * coefficientT);

        }
    }
}


TEST(testBAOptimized, visualizationDesk98) {

    gdr::ParamsRANSAC paramsRansacDefault;
    paramsRansacDefault.setProjectionUsage(false);

    gdr::CameraRGBD structureIoCamera(583, 320, 583, 240);
    structureIoCamera.setDepthPixelDivider(1000.0);

    gdr::CameraRGBD kinectCamera(517.3, 318.6, 516.5, 255.3);
    kinectCamera.setDepthPixelDivider(5000.0);


    std::string assocFile = "assoc.txt";

//    testReconstruction("fr1_desk_full", 573, 1,
//                       0.04, 0.04,
//                       kinectCamera,
//                       paramsRansacDefault,
//                       assocFile);


//    testReconstruction("fr1_desk_short", 4,
//                       1, 0.04, 0.04,
//                       kinectCamera, paramsRansacDefault, assocFile);

//    testReconstruction("copyroom_multiplied_by_5", 20, 1,
//                       0.04, 0.04,
//                       structureIoCamera);



//    testReconstruction("plant_sampled_19_3", 19, 3,
//                       0.04, 0.04,
//                       kinectCamera,
//                       paramsRansacDefault,
//                       assocFile);


    testReconstruction("desk1_sampled_98_6", 98, 6,
                       0.04, 0.04,
                       kinectCamera,
                       paramsRansacDefault,
                       assocFile);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



