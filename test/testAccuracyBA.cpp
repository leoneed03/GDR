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

#include "poseGraph/PosesForEvaluation.h"

void testReconstruction(
        const std::string &datasetName,
        double errorTresholdR,
        double errorTresholdT,
        const gdr::CameraRGBD &cameraDefault = gdr::CameraRGBD(),
        const gdr::ParamsRANSAC &paramsRansac = gdr::ParamsRANSAC(),
        const std::string &assocFile = "",
        int numberOfIterations = 1,
        bool printToConsole = false,
        bool showVisualization3D = false,
        bool savePointCloudPly = false,
        double minCoefficientOfBiggestComponent = 0.5,
        double coefficientR = 1.2,
        double coefficientT = 1.2,
        double timeDiffThreshold = 0.02) {

    for (int iteration = 0; iteration < numberOfIterations; ++iteration) {

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

        int numberOfPosesInDataset = cgHandler.getNumberOfVertices();

        cgHandler.bfsDrawToFile(
                "../../tools/data/temp/" + datasetName + "_connectedComponents.dot");
        std::vector<std::unique_ptr<gdr::AbsolutePosesComputationHandler>> connectedComponentsPoseGraph =
                cgHandler.splitGraphToConnectedComponents();


        if (printToConsole) {

            std::cout << "Biggest component of size "
                      << connectedComponentsPoseGraph[0]->getNumberOfPoses() << std::endl;

            for (int componentNumber = 0; componentNumber < connectedComponentsPoseGraph.size(); ++componentNumber) {
                std::cout << " #component index by increment " << componentNumber << " of size "
                          << connectedComponentsPoseGraph[componentNumber]->getNumberOfPoses() << std::endl;
            }
        }

        auto &biggestComponent = connectedComponentsPoseGraph[0];
        biggestComponent->setRelativePosesFilePath("relativeRotationsFile_0.txt");

        const auto &poseFixed = biggestComponent->getVertices()[biggestComponent->getIndexFixedPose()];
        std::cout << "perform rotation averaging" << std::endl;
        std::vector<gdr::SO3> computedAbsoluteOrientationsNoRobust = biggestComponent->performRotationAveraging();

        std::cout << "perform rotation robust optimization" << std::endl;
        std::vector<gdr::SO3> computedAbsoluteOrientationsRobust = biggestComponent->performRotationRobustOptimization();

        std::cout << "perform translation averaging" << std::endl;
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = biggestComponent->performTranslationAveraging();

        {
            std::string outputNameIRLS =
                    "../../tools/data/temp/" + datasetName + "_posesBiggestComponent_IRLS_matched.txt";
            std::ofstream posesIRLS(outputNameIRLS);

            posesIRLS << biggestComponent->getPosesForEvaluation();
        }

        std::cout << "perform Bundle Adjustment" << std::endl;


        std::vector<gdr::SE3> bundleAdjustedPoses = biggestComponent->performBundleAdjustmentUsingDepth();

        {
            std::string outputNameBA =
                    "../../tools/data/temp/" + datasetName + "_posesBiggestComponent_BA_matched.txt";
            std::ofstream posesBA(outputNameBA);

            posesBA << biggestComponent->getPosesForEvaluation();
        }

        std::string absolutePosesGroundTruth = "../../data/" + datasetName + "/" + "groundtruth.txt";
        std::vector<gdr::PoseFullInfo> posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesGroundTruth);

        int indexFixedPose = biggestComponent->getIndexFixedPose();

        std::vector<double> timestampsToFind = biggestComponent->getPosesTimestamps();

        //fill information needed for evaluation
        std::vector<gdr::PoseFullInfo> posesFullInfoIRLS;
        std::vector<gdr::PoseFullInfo> posesFullInfoBA;

        {
            std::vector<gdr::SE3> irlsPoses = biggestComponent->getPosesSE3();
            for (int i = 0; i < irlsPoses.size(); ++i) {
                const auto &pose = irlsPoses[i];
                posesFullInfoIRLS.emplace_back(gdr::PoseFullInfo(timestampsToFind[i], pose));
            }

            for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
                const auto &pose = bundleAdjustedPoses[i];
                posesFullInfoBA.emplace_back(gdr::PoseFullInfo(timestampsToFind[i], pose));
            }
        }

        assert(posesFullInfoBA.size() == posesFullInfoIRLS.size());
        assert(!posesFullInfoIRLS.empty());
        assert(posesFullInfoIRLS.size() == timestampsToFind.size());

        //sizes may be not equal with posesInfoFullBA!!!
        posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientationByMatches(posesInfoFull,
                                                                                       timestampsToFind,
                                                                                       timeDiffThreshold);
        gdr::SE3 fixedPoseGroundTruth(posesInfoFull[0].getSophusPose());
        double minTimeDiff = std::numeric_limits<double>::max();


        for (const auto &poseGroundTruth: posesInfoFull) {

            double currentTimeDiff = std::abs(poseGroundTruth.getTimestamp() - timestampsToFind[indexFixedPose]);

            if (currentTimeDiff < minTimeDiff) {
                fixedPoseGroundTruth = gdr::SE3(poseGroundTruth.getSophusPose());
            }

            minTimeDiff = std::min(minTimeDiff, currentTimeDiff);
        }


        {
            std::string outputNameGroundTruth =
                    "../../tools/data/temp/" + datasetName + "_posesBiggestComponent_GT_matched.txt";
            std::ofstream groundTruthPoses(outputNameGroundTruth);
            groundTruthPoses << gdr::PosesForEvaluation(
                    posesInfoFull,
                    fixedPoseGroundTruth.inverse());
        }

        gdr::Evaluator evaluator(absolutePosesGroundTruth);

        double meanErrorRotBA = 0;
        double meanErrorL2BA = 0;

        double meanErrorRotIRLS = 0;
        double meanErrorL2IRLS = 0;

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

            std::cout << std::endl << "Trajectory estimated for: " << informationErrors.numberOfPosesTrajectory << "/"
                      << numberOfPosesInDataset << " poses" << std::endl;
            std::cout << "Compared with groundtruth: " << informationErrors.numberOfPosesEvaluated << "/"
                      << informationErrors.numberOfPosesTrajectory << std::endl;
        }


        if (showVisualization3D) {
            gdr::ModelCreationHandler modelCreationHandler(biggestComponent->getPoseGraph());
            modelCreationHandler.visualize();
        }

        if (savePointCloudPly) {
            gdr::ModelCreationHandler modelCreationHandler(biggestComponent->getPoseGraph());
            modelCreationHandler.saveAsPly("test.ply");
        }


        ASSERT_GE(bundleAdjustedPoses.size(), numberOfPosesInDataset * minCoefficientOfBiggestComponent);

        ASSERT_LE(meanErrorL2BA, errorTresholdT);
        ASSERT_LE(meanErrorL2IRLS, errorTresholdT);
        ASSERT_LE(meanErrorRotBA, errorTresholdR);

        ASSERT_LE(meanErrorRotBA, meanErrorRotIRLS * coefficientR);
        ASSERT_LE(meanErrorL2BA, meanErrorL2IRLS * coefficientT);
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

    testReconstruction("plant_sampled_19_3",
                       0.04, 0.04,
                       kinectCamera,
                       paramsRansacDefault,
                       assocFile);

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



