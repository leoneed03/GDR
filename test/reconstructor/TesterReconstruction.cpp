//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "directoryTraversing/DirectoryReader.h"
#include "TesterReconstruction.h"

#include "readerDataset/readerTUM/ReaderTum.h"
#include "boost/filesystem.hpp"

namespace test {

    namespace fs = boost::filesystem;

    ErrorsOfTrajectoryEstimation TesterReconstruction::testReconstruction(
            const std::string &pathRelativeToData,
            const gdr::CameraRGBD &cameraDefault,
            const gdr::ParamsRANSAC &paramsRansac,
            const std::string &assocFile,
            const std::string &pathOutPoses,
            const OutputShortFileNames &outputShortFileNames,
            double timeDiffThreshold,
            bool printToConsole,
            bool showVisualization3D,
            bool savePointCloudPly,
            const std::vector<int> &gpuDevices,
            bool printFullReport) {

        ErrorsOfTrajectoryEstimation errorsOfTrajectoryEstimation;


        std::cout << "Running test on " << pathRelativeToData << std::endl;


        const fs::path datasetPath(pathRelativeToData);
        std::string shortDatasetName = datasetPath.filename().string();

        gdr::DatasetStructure datasetStructure = gdr::ReaderTUM::getDatasetStructure(datasetPath.string(),
                                                                                     assocFile);
        std::vector<std::unique_ptr<gdr::AbsolutePosesComputationHandler>> connectedComponentsPoseGraph;
        int numberOfPosesInDataset = 0;

        std::stringstream benchmarkInfoRelativePoses;
        {
            gdr::RelativePosesComputationHandler cgHandler(
                    gdr::DatasetDescriber(datasetStructure,
                                          cameraDefault),
                    paramsRansac);

            if (printToConsole) {
                std::cout << "start computing relative poses" << std::endl;
            }
            cgHandler.computeRelativePoses(gpuDevices);
            benchmarkInfoRelativePoses = cgHandler.getTimeBenchmarkInfo();

            numberOfPosesInDataset = cgHandler.getNumberOfVertices();
            errorsOfTrajectoryEstimation.numberOfPosesInDataset = numberOfPosesInDataset;

            connectedComponentsPoseGraph = cgHandler.splitGraphToConnectedComponents();
        }


        if (printToConsole) {

            std::cout << "Biggest component of size "
                      << connectedComponentsPoseGraph[0]->getNumberOfPoses() << std::endl;

            for (int componentNumber = 0;
                 componentNumber < connectedComponentsPoseGraph.size(); ++componentNumber) {
                std::cout << " #component index by increment " << componentNumber << " of size "
                          << connectedComponentsPoseGraph[componentNumber]->getNumberOfPoses() << std::endl;
            }
        }

        auto &biggestComponent = connectedComponentsPoseGraph[0];
        biggestComponent->setRelativePosesFilePath("relativeRotationsFile_0.txt");

        const auto &poseFixed = biggestComponent->getVertices()[biggestComponent->getIndexFixedPose()];

        if (printToConsole) {
            std::cout << "perform rotation averaging" << std::endl;
        }
        std::vector<gdr::SO3> computedAbsoluteOrientationsNoRobust = biggestComponent->performRotationAveraging();

        if (printToConsole) {
            std::cout << "perform rotation robust optimization" << std::endl;
        }
        std::vector<gdr::SO3> computedAbsoluteOrientationsRobust = biggestComponent->performRotationRobustOptimization();

        if (printToConsole) {
            std::cout << "perform translation averaging" << std::endl;
        }
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = biggestComponent->performTranslationAveraging();

        std::string fullOutPath;

        if (!pathOutPoses.empty()) {
            fs::path fsPathOut(pathOutPoses);
            fullOutPath = fsPathOut.string();
        }
        std::vector<gdr::SE3> irlsPoses = biggestComponent->getPosesSE3();

        {
            std::string outputNameIRLS = gdr::DirectoryReader::appendPathSuffix(fullOutPath,
                                                                                outputShortFileNames.posesIRLS);
            if (printToConsole) {
                std::cout << "IRLS poses written to: " << outputNameIRLS << std::endl;
            }
            std::ofstream posesIRLS(outputNameIRLS);

            posesIRLS << biggestComponent->getPosesForEvaluation();
        }

        if (printToConsole) {
            std::cout << "perform Bundle Adjustment" << std::endl;
        }

        std::vector<gdr::SE3> bundleAdjustedPoses = biggestComponent->performBundleAdjustmentUsingDepth();

        std::stringstream benchmarkTimeInfoAbsolutePoses = biggestComponent->printTimeBenchmarkInfo();

        {
            std::string outputNameBA =
                    gdr::DirectoryReader::appendPathSuffix(fullOutPath, outputShortFileNames.posesBA);
            std::ofstream posesBA(outputNameBA);

            if (printToConsole) {
                std::cout << "BA poses written to: " << outputNameBA << std::endl;
            }

            posesBA << biggestComponent->getPosesForEvaluation();
        }

        std::string absolutePosesGroundTruth = gdr::DirectoryReader::appendPathSuffix(pathRelativeToData,
                                                                                      "groundtruth.txt");
        std::vector<gdr::PoseFullInfo> posesInfoFull = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesGroundTruth);

        int indexFixedPose = biggestComponent->getIndexFixedPose();

        std::vector<double> timestampsToFind = biggestComponent->getPosesTimestamps();

        //fill information needed for evaluation
        std::vector<gdr::PoseFullInfo> posesFullInfoIRLS;
        std::vector<gdr::PoseFullInfo> posesFullInfoBA;

        {
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
        if (printToConsole) {
            std::cout << "found timestamp matches: " << posesInfoFull.size() << std::endl;
        }
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
                    gdr::DirectoryReader::appendPathSuffix(fullOutPath, outputShortFileNames.posesGroundTruth);
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
            {
                auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoBA,
                                                                      biggestComponent->getIndexFixedPose(),
                                                                      true);
                if (printFullReport) {
                    std::cout << "========================BA report[Umeyama ALIGNED]:========================="
                              << std::endl;
                    std::cout << informationErrors << std::endl;
                }

                errorsOfTrajectoryEstimation.errorAlignedUmeyamaBA = informationErrors;
            }
            if (printFullReport) {
                std::cout << "------------------------------------------------------------------------------------"
                          << std::endl;
            }

            {
                auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoBA,
                                                                      biggestComponent->getIndexFixedPose(),
                                                                      false);
                if (printFullReport) {
                    std::cout << "========================BA report[Fixed Pose ALIGNED]:========================="
                              << std::endl;
                    std::cout << informationErrors << std::endl;
                }
                meanErrorRotBA = informationErrors.rotationError.MEAN;
                meanErrorL2BA = informationErrors.translationError.MEAN;

                errorsOfTrajectoryEstimation.errorBA = informationErrors;
            }

            if (printFullReport) {
                std::cout << std::endl << std::endl;
            }

            assert(!posesFullInfoIRLS.empty());

            {
                auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoIRLS,
                                                                      biggestComponent->getIndexFixedPose(),
                                                                      true);

                if (printFullReport) {
                    std::cout << "========================IRLS report [Umeyama Aligned]:========================="
                              << std::endl;
                    std::cout << informationErrors << std::endl;
                }
                errorsOfTrajectoryEstimation.errorAlignedUmeyamaIRLS = informationErrors;
            }
            if (printFullReport) {
                std::cout << "------------------------------------------------------------------------------------"
                          << std::endl;
            }

            {
                auto informationErrors = evaluator.evaluateTrajectory(posesFullInfoIRLS,
                                                                      biggestComponent->getIndexFixedPose(),
                                                                      false);
                if (printFullReport) {
                    std::cout << "========================IRLS report [Fixed Pose Aligned]:========================="
                              << std::endl;
                    std::cout << informationErrors << std::endl;
                }

                meanErrorL2IRLS = informationErrors.translationError.MEAN;
                meanErrorRotIRLS = informationErrors.rotationError.MEAN;

                errorsOfTrajectoryEstimation.errorIRLS = informationErrors;

                if (printFullReport) {
                    std::cout << "Trajectory estimated for: " << informationErrors.numberOfPosesTrajectory << "/"
                              << numberOfPosesInDataset << " poses" << std::endl;
                }
            }

        }

        if (showVisualization3D) {
            gdr::ModelCreationHandler modelCreationHandler(biggestComponent->getPoseGraph());
            modelCreationHandler.visualize();
        }

        if (savePointCloudPly) {
            gdr::ModelCreationHandler modelCreationHandler(biggestComponent->getPoseGraph());
            modelCreationHandler.saveAsPly("test.ply");
        }

        if (printFullReport) {
            std::cout << benchmarkInfoRelativePoses.str();
            std::cout << benchmarkTimeInfoAbsolutePoses.str() << std::endl;
        }

        return errorsOfTrajectoryEstimation;
    }
}
