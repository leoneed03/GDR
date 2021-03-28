//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <boost/filesystem.hpp>
#include "absolutePoseEstimation/rotationAveraging/rotationAveraging.h"
#include "absolutePoseEstimation/rotationAveraging/RotationOptimizationRobust.h"
#include "absolutePoseEstimation/translationAveraging/translationMeasurement.h"
#include "absolutePoseEstimation/rotationAveraging/rotationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/translationAveraging.h"

#include "bundleAdjustment/IBundleAdjuster.h"
#include "bundleAdjustment/BundleAdjuster.h"

#include "sparsePointCloud/CloudProjector.h"
#include "sparsePointCloud/ProjectableInfo.h"

#include "computationHandlers/AbsolutePosesComputationHandler.h"

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

        cloudProjector = std::make_unique<CloudProjector>(posesForCloudProjector);

        const auto &matchesBetweenPoints = connectedComponent->getInlierObservedPoints();
        for (const auto &vectorOfMatches: matchesBetweenPoints) {

            std::vector<std::pair<int, int>> poseAndLocalIndices;
            for (const std::pair<std::pair<int, int>, KeyPointInfo> &fullPointInfo: vectorOfMatches) {
                poseAndLocalIndices.push_back(fullPointInfo.first);
            }
            pointMatcher->insertPointsWithNewClasses(poseAndLocalIndices);
        }


        // unordered map's Key is local index
        std::vector<std::unordered_map<int, KeyPointInfo>>
                keyPointInfoByPoseNumAndLocalInd(pointMatcher->getNumberOfPoses());


        for (const auto &vectorOfMatches: matchesBetweenPoints) {

            for (const std::pair<std::pair<int, int>, KeyPointInfo> &fullPointInfo: vectorOfMatches) {
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
            keyPointInfo.push_back(
                    keyPointInfoByPoseNumAndLocalInd[poseNumberAndLocalIndex.first][poseNumberAndLocalIndex.second]);
            cloudProjector->addPoint(pointClassNumber, keyPointInfo);
        }
    }

    int AbsolutePosesComputationHandler::getNumberOfPoses() const {
        return connectedComponent->getNumberOfPoses();
    }

    std::vector<SO3> AbsolutePosesComputationHandler::performRotationAveraging() {

        connectedComponent->printRelativeRotationsToFile(connectedComponent->getPathRelativePoseFile());

        std::vector<SO3> absoluteRotations = rotationAverager::shanonAveraging(
                connectedComponent->getPathRelativePoseFile(),
                connectedComponent->getPathAbsoluteRotationsFile());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setRotation(i, SO3(absoluteRotations[i].getRotationSophus()));
        }

        return absoluteRotations;
    }

    std::vector<SO3> AbsolutePosesComputationHandler::optimizeRotationsRobust() {


        std::vector<SO3> shonanOptimizedAbsolutePoses;

        for (const auto &vertexPose: connectedComponent->getVertices()) {
            shonanOptimizedAbsolutePoses.emplace_back(SO3(vertexPose.getRotationQuat()));
        }

        assert(shonanOptimizedAbsolutePoses.size() == getNumberOfPoses());


        std::vector<rotationMeasurement> relativeRotationsAfterICP;

        for (int indexFrom = 0; indexFrom < getNumberOfPoses(); ++indexFrom) {
            for (const auto &knownRelativePose: connectedComponent->getConnectionsFromVertex(indexFrom)) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {
                    relativeRotationsAfterICP.emplace_back(
                            rotationMeasurement(knownRelativePose.getRelativeRotation(),
                                                knownRelativePose.getIndexFrom(),
                                                knownRelativePose.getIndexTo()));
                }
            }
        }

        RotationOptimizer rotationOptimizer(shonanOptimizedAbsolutePoses, relativeRotationsAfterICP);
        std::vector<SO3> optimizedPosesRobust = rotationOptimizer.getOptimizedOrientation();

        assert(getNumberOfPoses() == optimizedPosesRobust.size());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setRotation(i, optimizedPosesRobust[i]);
        }

        return optimizedPosesRobust;

    }

    std::vector<Eigen::Vector3d> AbsolutePosesComputationHandler::optimizeAbsoluteTranslations(int indexFixedToZero) {

        std::vector<translationMeasurement> relativeTranslations;
        std::vector<SE3> absolutePoses = connectedComponent->getPoses();

        for (int indexFrom = 0; indexFrom < getNumberOfPoses(); ++indexFrom) {
            for (const auto &knownRelativePose: connectedComponent->getConnectionsFromVertex(indexFrom)) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {
                    relativeTranslations.emplace_back(
                            translationMeasurement(knownRelativePose.getRelativeTranslation(),
                                                   knownRelativePose.getIndexFrom(),
                                                   knownRelativePose.getIndexTo()));
                }
            }
        }

        std::vector<Eigen::Vector3d> optimizedAbsoluteTranslationsIRLS = translationAverager::recoverTranslations(
                relativeTranslations,
                absolutePoses).toVectorOfVectors();

        bool successIRLS = true;

        // Now run IRLS with PCG answer as init solution

        optimizedAbsoluteTranslationsIRLS = translationAverager::recoverTranslationsIRLS(
                relativeTranslations,
                absolutePoses,
                optimizedAbsoluteTranslationsIRLS,
                successIRLS).toVectorOfVectors();


        Eigen::Vector3d zeroTranslation = optimizedAbsoluteTranslationsIRLS[indexFixedToZero];
        for (auto &translation: optimizedAbsoluteTranslationsIRLS) {
            translation -= zeroTranslation;
        }

        assert(getNumberOfPoses() == optimizedAbsoluteTranslationsIRLS.size());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setTranslation(i, optimizedAbsoluteTranslationsIRLS[i]);
        }

        return optimizedAbsoluteTranslationsIRLS;
    }

    std::vector<SE3> AbsolutePosesComputationHandler::performBundleAdjustmentUsingDepth(int indexFixedToZero) {
        int maxNumberOfPointsToShow = -1;
        computePointClasses();
        std::vector<Point3d> observedPoints = cloudProjector->setComputedPointsGlobalCoordinates();
        std::cout << "ready " << std::endl;

        std::vector<std::pair<SE3, CameraRGBD>> posesAndCameraParams;

        assert(!connectedComponent->getVertices().empty());
        for (const auto &vertexPose: connectedComponent->getVertices()) {
            posesAndCameraParams.emplace_back(std::make_pair(vertexPose.getAbsolutePoseSE3(), vertexPose.getCamera()));
        }
        std::cout << "BA depth create BundleAdjuster" << std::endl;

        std::vector<double> errorsBefore;
        auto shownResidualsBefore = cloudProjector->showPointsReprojectionError(observedPoints,
                                                                                "before",
                                                                                errorsBefore,
                                                                                connectedComponent->getVertex(
                                                                                        0).getCamera(),
                                                                                maxNumberOfPointsToShow);

        std::unique_ptr<IBundleAdjuster> bundleAdjuster =
                std::make_unique<BundleAdjuster>(observedPoints,
                                                 posesAndCameraParams,
                                                 cloudProjector->getKeyPointInfoByPoseNumberAndPointClass());

        std::vector<SE3> posesOptimized = bundleAdjuster->optimizePointsAndPosesUsingDepthInfo(indexFixedToZero);


        assert(posesOptimized.size() == getNumberOfPoses());

        for (int i = 0; i < getNumberOfPoses(); ++i) {
            connectedComponent->setPoseSE3(i, posesOptimized[i]);
        }

        std::vector<double> errorsAfter;

        cloudProjector->setPoses(connectedComponent->getPoses());
        cloudProjector->setPoints(bundleAdjuster->getOptimizedPoints());

        auto shownResidualsAfter = cloudProjector->showPointsReprojectionError(observedPoints,
                                                                               "after",
                                                                               errorsAfter,
                                                                               connectedComponent->getVertex(
                                                                                       0).getCamera(),
                                                                               maxNumberOfPointsToShow);

        assert(shownResidualsAfter.size() == shownResidualsBefore.size());


        std::string pathToRGBDirectoryToSave = "shownResiduals";
        boost::filesystem::path pathToRemove(pathToRGBDirectoryToSave);

        std::cout << "path [" << pathToRemove.string() << "]" << " exists? Answer: "
                  << boost::filesystem::exists(pathToRemove) << std::endl;
        boost::filesystem::remove_all(pathToRemove);
        std::cout << "removed from " << pathToRemove.string() << std::endl;
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

        std::cout << "BETTER #median error: " << counterMedianErrorGotBetter
                  << " vs WORSE: " << counterMedianErrorGotWorse << " of total " << counterSumMedianError << std::endl;
        std::cout << "percentage better median error is "
                  << 1.0 * counterMedianErrorGotBetter / counterSumMedianError << std::endl;

        assert(counterMedianErrorGotWorse + counterMedianErrorGotBetter == counterSumMedianError);

        return posesOptimized;
    }

    AbsolutePosesComputationHandler::AbsolutePosesComputationHandler(
            std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph) {

        connectedComponent = std::move(connectedComponentPoseGraph);
        pointMatcher = std::make_unique<PointClassifier>(getNumberOfPoses());
    }

    std::set<int> AbsolutePosesComputationHandler::initialIndices() const {
        return connectedComponent->initialIndices();
    }

    const std::vector<VertexCG> &AbsolutePosesComputationHandler::getVertices() const {
        return connectedComponent->getVertices();
    }

}
