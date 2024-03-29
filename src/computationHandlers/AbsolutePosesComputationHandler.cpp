//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <boost/filesystem.hpp>
#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerCreator.h"
#include "absolutePoseEstimation/translationAveraging/TranslationMeasurement.h"
#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"

#include "bundleAdjustment/BundleAdjuster.h"
#include "bundleAdjustment/BundleAdjusterCreator.h"

#include "sparsePointCloud/CloudProjectorCreator.h"
#include "sparsePointCloud/PointClassifierCreator.h"
#include "sparsePointCloud/ProjectableInfo.h"

#include "computationHandlers/AbsolutePosesComputationHandler.h"
#include "computationHandlers/TimerClockNow.h"

namespace gdr {

    void AbsolutePosesComputationHandler::computePointClasses() {

        std::vector<ProjectableInfo> posesForCloudProjector;
        posesForCloudProjector.reserve(getNumberOfPoses());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            const auto &pose = connectedComponent->getVertex(i);
            posesForCloudProjector.emplace_back(
                    ProjectableInfo(
                            pose.getAbsolutePoseSE3(),
                            pose.getCamera(),
                            pose.getIndex(),
                            pose.getPathRGBImage(),
                            pose.getPathDImage()
                    ));
        }

        cloudProjector = CloudProjectorCreator::getCloudProjector();
        cloudProjector->setCameraPoses(posesForCloudProjector);

        const auto &matchesBetweenPoints = connectedComponent->getInlierObservedPoints();
        for (const auto &vectorOfMatches: matchesBetweenPoints) {

            std::vector<std::pair<int, int>> poseAndLocalIndices;
            poseAndLocalIndices.push_back(vectorOfMatches.first.first);
            poseAndLocalIndices.push_back(vectorOfMatches.second.first);

            pointMatcher->insertPointsWithNewClasses(poseAndLocalIndices);
        }


        // unordered map's Key is local index
        std::vector<std::unordered_map<int, KeyPointInfo>>
                keyPointInfoByPoseNumAndLocalInd(pointMatcher->getNumberOfPoses());


        for (const auto &vectorOfMatches: matchesBetweenPoints) {

            for (int i = 0; i < 2; ++i) {
                const auto &fullPointInfo = (i == 0) ? (vectorOfMatches.first) : (vectorOfMatches.second);

                const auto &poseNumAndLocalInd = fullPointInfo.first;
                const auto &foundIt = keyPointInfoByPoseNumAndLocalInd[poseNumAndLocalInd.first].find(
                        poseNumAndLocalInd.second);
                if (foundIt != keyPointInfoByPoseNumAndLocalInd[poseNumAndLocalInd.first].end()) {
                    assert(foundIt->second == fullPointInfo.second);
                } else {
                    keyPointInfoByPoseNumAndLocalInd[poseNumAndLocalInd.first].insert(
                            std::make_pair(poseNumAndLocalInd.second, fullPointInfo.second));
                }
            }
        }

        auto pointClasses = pointMatcher->assignPointClasses();

        for (int pointIncrementor = 0; pointIncrementor < pointClasses.size(); ++pointIncrementor) {
            int pointClassNumber = pointClasses[pointIncrementor];
            std::pair<int, int> poseNumberAndLocalIndex = pointMatcher->getPoseNumberAndLocalIndex(pointIncrementor);
            std::vector<KeyPointInfo> keyPointInfo;
            keyPointInfo.emplace_back(
                    keyPointInfoByPoseNumAndLocalInd[poseNumberAndLocalIndex.first][poseNumberAndLocalIndex.second]);
            cloudProjector->addPoint(pointClassNumber, keyPointInfo);
        }
    }

    int AbsolutePosesComputationHandler::getNumberOfPoses() const {
        return connectedComponent->getNumberOfPoses();
    }


    std::vector<RotationMeasurement> AbsolutePosesComputationHandler::getRelativeRotationsVector() const {

        std::vector<RotationMeasurement> relativeRotationsToReturn;

        for (int indexFromDest = 0; indexFromDest < getPoseGraph().size(); ++indexFromDest) {
            for (const auto &relativePose: getPoseGraph().getRelativePosesFrom(indexFromDest)) {

                int indexToToBeTransformed = relativePose.getIndexTo();

                assert(indexFromDest == relativePose.getIndexFrom());

                relativeRotationsToReturn.emplace_back(RotationMeasurement(
                        relativePose.getRelativeRotation(), indexFromDest, indexToToBeTransformed
                ));
            }
        }

        return relativeRotationsToReturn;
    }

    std::vector<SO3> AbsolutePosesComputationHandler::performRotationAveraging() {

        timeStartRotationAveraging = timerGetClockTimeNow();
        std::vector<SO3> absoluteRotations = RotationAverager::shanonAveraging(
                getRelativeRotationsVector(),
                getIndexFixedPose(),
                getPathRelativePoseFile());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setRotation(i, SO3(absoluteRotations[i].getRotationSophus()));
        }

        timeEndRotationAveraging = timerGetClockTimeNow();

        return absoluteRotations;
    }

    std::vector<SO3> AbsolutePosesComputationHandler::performRotationRobustOptimization() {


        std::vector<SO3> shonanOptimizedAbsolutePoses;

        for (const auto &vertexPose: connectedComponent->getVertices()) {
            shonanOptimizedAbsolutePoses.emplace_back(SO3(vertexPose.getRotationQuat()));
        }

        assert(shonanOptimizedAbsolutePoses.size() == getNumberOfPoses());


        std::vector<RotationMeasurement> relativeRotationsAfterICP;

        for (int indexFrom = 0; indexFrom < getNumberOfPoses(); ++indexFrom) {
            for (const auto &knownRelativePose: connectedComponent->getConnectionsFromVertex(indexFrom)) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {
                    relativeRotationsAfterICP.emplace_back(
                            RotationMeasurement(knownRelativePose.getRelativeRotation(),
                                                knownRelativePose.getIndexFrom(),
                                                knownRelativePose.getIndexTo()));
                }
            }
        }

        timeStartRobustRotationOptimization = timerGetClockTimeNow();
        std::unique_ptr<RotationRobustOptimizer> rotationOptimizer =
                RotationRobustOptimizerCreator::getRefiner(
                        RotationRobustOptimizerCreator::RobustParameterType::DEFAULT);

        std::vector<SO3> optimizedPosesRobust =
                rotationOptimizer->getOptimizedOrientation(shonanOptimizedAbsolutePoses,
                                                           relativeRotationsAfterICP,
                                                           connectedComponent->getPoseIndexWithMaxConnectivity());
        timeEndRobustRotationOptimization = timerGetClockTimeNow();

        assert(getNumberOfPoses() == optimizedPosesRobust.size());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setRotation(i, optimizedPosesRobust[i]);
        }

        return optimizedPosesRobust;

    }

    std::vector<Eigen::Vector3d> AbsolutePosesComputationHandler::performTranslationAveraging() {

        std::vector<TranslationMeasurement> relativeTranslations;
        std::vector<SE3> absolutePoses = connectedComponent->getPoses();
        int indexFixedToZero = connectedComponent->getPoseIndexWithMaxConnectivity();

        for (int indexFrom = 0; indexFrom < getNumberOfPoses(); ++indexFrom) {
            for (const auto &knownRelativePose: connectedComponent->getConnectionsFromVertex(indexFrom)) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {
                    relativeTranslations.emplace_back(
                            TranslationMeasurement(knownRelativePose.getRelativeTranslation(),
                                                   knownRelativePose.getIndexFrom(),
                                                   knownRelativePose.getIndexTo()));
                }
            }
        }

        timeStartTranslationAveraging = timerGetClockTimeNow();

        std::vector<Eigen::Vector3d> optimizedAbsoluteTranslationsIRLS = TranslationAverager::recoverTranslations(
                relativeTranslations,
                absolutePoses,
                indexFixedToZero).toVectorOfVectors();

        assert(optimizedAbsoluteTranslationsIRLS[indexFixedToZero].norm() < std::numeric_limits<double>::epsilon());

        bool successIRLS = true;

        // Now run IRLS with PCG answer as init solution
        optimizedAbsoluteTranslationsIRLS = TranslationAverager::recoverTranslationsIRLS(
                relativeTranslations,
                absolutePoses,
                optimizedAbsoluteTranslationsIRLS,
                indexFixedToZero,
                successIRLS).toVectorOfVectors();

        timeEndTranslationAveraging = timerGetClockTimeNow();

        assert(optimizedAbsoluteTranslationsIRLS[indexFixedToZero].norm() < std::numeric_limits<double>::epsilon());

        assert(getNumberOfPoses() == optimizedAbsoluteTranslationsIRLS.size());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setTranslation(i, optimizedAbsoluteTranslationsIRLS[i]);
        }

        return optimizedAbsoluteTranslationsIRLS;
    }

    std::vector<SE3> AbsolutePosesComputationHandler::performBundleAdjustmentUsingDepth() {
        int maxNumberOfPointsToShow = -1;
        computePointClasses();

        int indexFixedToZero = connectedComponent->getPoseIndexWithMaxConnectivity();
        std::vector<Point3d> observedPoints = cloudProjector->computedPointsGlobalCoordinates();
        std::vector<std::pair<SE3, CameraRGBD>> posesAndCameraParams;

        assert(!connectedComponent->getVertices().empty());
        for (const auto &vertexPose: connectedComponent->getVertices()) {
            posesAndCameraParams.emplace_back(std::make_pair(vertexPose.getAbsolutePoseSE3(), vertexPose.getCamera()));
        }

        std::vector<double> errorsBefore;
        std::vector<cv::Mat> shownResidualsBefore;


        if (saveDebugImages) {
            shownResidualsBefore = cloudProjector->showPointsReprojectionError(observedPoints,
                                                                               "before",
                                                                               errorsBefore,
                                                                               connectedComponent->getVertex(
                                                                                       0).getCamera(),
                                                                               maxNumberOfPointsToShow);
        }

        timeStartBundleAdjustment = timerGetClockTimeNow();

        std::unique_ptr<BundleAdjuster> bundleAdjuster =
                BundleAdjusterCreator::getBundleAdjuster(BundleAdjusterCreator::BundleAdjustmentType::USE_DEPTH_INFO);

        bool isUsableBA = true;
        std::vector<SE3> posesOptimized = bundleAdjuster->optimizePointsAndPoses(observedPoints,
                                                                                 posesAndCameraParams,
                                                                                 cloudProjector->getKeyPointInfoByPoseNumberAndPointClass(),
                                                                                 indexFixedToZero,
                                                                                 isUsableBA);

        timeEndBundleAdjustment = timerGetClockTimeNow();

        assert(posesOptimized.size() == getNumberOfPoses());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setPoseSE3(i, posesOptimized[i]);
        }

        std::vector<double> errorsAfter;

        cloudProjector->setPoses(connectedComponent->getPoses());
        cloudProjector->setPoints(bundleAdjuster->getOptimizedPoints());

        if (saveDebugImages) {
            auto shownResidualsAfter = cloudProjector->showPointsReprojectionError(observedPoints,
                                                                                   "after",
                                                                                   errorsAfter,
                                                                                   connectedComponent->getVertex(
                                                                                           0).getCamera(),
                                                                                   maxNumberOfPointsToShow);

            assert(shownResidualsAfter.size() == shownResidualsBefore.size());


            std::string pathToRGBDirectoryToSave = "shownResiduals";
            boost::filesystem::path pathToRemove(pathToRGBDirectoryToSave);

            boost::filesystem::remove_all(pathToRemove);
            boost::filesystem::create_directories(pathToRemove);


            int counterMedianErrorGotWorse = 0;
            int counterMedianErrorGotBetter = 0;
            int counterSumMedianError = 0;

            for (int i = 0; i < shownResidualsAfter.size(); ++i) {

                boost::filesystem::path pathToSave = pathToRemove;
                bool medianErrorGotLessAfterBA = (errorsBefore[i] > errorsAfter[i]);

                if (medianErrorGotLessAfterBA) {
                    ++counterMedianErrorGotBetter;
                } else {
                    ++counterMedianErrorGotWorse;
                }
                ++counterSumMedianError;

                std::string betterOrWorseResults = medianErrorGotLessAfterBA ? " " : " [WORSE] ";
                std::string nameImageReprojErrors =
                        std::to_string(i) + betterOrWorseResults + " quantils: " + std::to_string(errorsBefore[i]) +
                        " -> " + std::to_string(errorsAfter[i]) + ".png";
                pathToSave.append(nameImageReprojErrors);
                cv::Mat stitchedImageResiduals;
                std::vector<cv::DMatch> matches1to2;
                std::vector<cv::KeyPoint> keyPointsToShowFirst;
                std::vector<cv::KeyPoint> keyPointsToShowSecond;
                cv::Mat stitchedImage;
                cv::drawMatches(shownResidualsBefore[i], {},
                                shownResidualsAfter[i], {},
                                matches1to2,
                                stitchedImage);
                cv::imwrite(pathToSave.string(), stitchedImage);
            }

            assert(counterMedianErrorGotWorse + counterMedianErrorGotBetter == counterSumMedianError);
        }

        return posesOptimized;
    }

    AbsolutePosesComputationHandler::AbsolutePosesComputationHandler(
            std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph) {

        connectedComponent = std::move(connectedComponentPoseGraph);
        pointMatcher = PointClassifierCreator::getPointClassifier();
        pointMatcher->setNumberOfPoses(getNumberOfPoses());
    }

    std::set<int> AbsolutePosesComputationHandler::initialIndices() const {
        return connectedComponent->initialIndices();
    }

    const std::vector<VertexPose> &AbsolutePosesComputationHandler::getVertices() const {
        return connectedComponent->getVertices();
    }

    bool AbsolutePosesComputationHandler::getSaveDebugImages() const {
        return saveDebugImages;
    }

    void AbsolutePosesComputationHandler::setSaveDebugImages(bool saveImages) {
        saveDebugImages = saveImages;
    }

    std::vector<double> AbsolutePosesComputationHandler::getPosesTimestamps() const {
        std::vector<double> timestampsToReturn;

        const auto &vertices = getVertices();

        for (const auto &vertex: vertices) {
            timestampsToReturn.emplace_back(vertex.getTimestamp());
        }

        assert(timestampsToReturn.size() == getNumberOfPoses());
        assert(!timestampsToReturn.empty());

        return timestampsToReturn;
    }

    std::vector<SE3> AbsolutePosesComputationHandler::getPosesSE3() const {
        std::vector<SE3> posesSE3;

        for (const auto &vertex: getVertices()) {
            posesSE3.emplace_back(vertex.getAbsolutePoseSE3());
        }

        assert(posesSE3.size() == getNumberOfPoses());
        return posesSE3;
    }


    const PoseGraph &AbsolutePosesComputationHandler::getPoseGraph() const {
        return connectedComponent->getPoseGraph();
    }


    int AbsolutePosesComputationHandler::getIndexFixedPose() const {
        return connectedComponent->getPoseIndexWithMaxConnectivity();
    }


    PosesForEvaluation AbsolutePosesComputationHandler::getPosesForEvaluation() const {
        return connectedComponent->getPosesForEvaluation();
    }

    PosesForEvaluation AbsolutePosesComputationHandler::getPosesForEvaluation(const SE3 &poseFixedInversed) const {
        return connectedComponent->getPosesForEvaluation(poseFixedInversed);
    }


    void AbsolutePosesComputationHandler::setRelativePosesFilePath(const std::string &pathRelPoses) {
        pathRelativePosesFile = pathRelPoses;
    }

    std::string AbsolutePosesComputationHandler::getPathRelativePoseFile() const {
        return pathRelativePosesFile;
    }

    std::stringstream AbsolutePosesComputationHandler::printTimeBenchmarkInfo() const {
        std::chrono::duration<double> timeRotationAveraging = std::chrono::duration_cast<std::chrono::duration<double>>(
                timeEndRotationAveraging - timeStartRotationAveraging);
        std::chrono::duration<double> timeRotationRobust = std::chrono::duration_cast<std::chrono::duration<double>>(
                timeEndRobustRotationOptimization - timeStartRobustRotationOptimization);
        std::chrono::duration<double> timeTranslationAveraging = std::chrono::duration_cast<std::chrono::duration<double>>(
                timeEndTranslationAveraging - timeStartTranslationAveraging);
        std::chrono::duration<double> timeBundleAdjustment = std::chrono::duration_cast<std::chrono::duration<double>>(
                timeEndBundleAdjustment - timeStartBundleAdjustment);

        std::stringstream benchmarkTimeInfo;

        benchmarkTimeInfo << "          Rotation Averaging: " << timeRotationAveraging.count() << std::endl;
        benchmarkTimeInfo << "          Robust Rotation Optimization: " << timeRotationRobust.count() << std::endl;
        benchmarkTimeInfo << "          Translation Averaging: " << timeTranslationAveraging.count() << std::endl;
        benchmarkTimeInfo << "          Bundle Adjustment: " << timeBundleAdjustment.count() << std::endl;

        return benchmarkTimeInfo;
    }
}
