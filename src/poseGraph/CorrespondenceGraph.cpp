//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "directoryTraversing/DirectoryReader.h"
#include "poseGraph/CorrespondenceGraph.h"
#include "printer.h"
#include "relativePoseRefinement/ICP.h"
#include "visualization/2D/ImageDrawer.h"
#include "keyPointDetectionAndMatching/SiftModuleGPU.h"
#include "relativePoseEstimators/EstimatorRobustLoRANSAC.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <tbb/parallel_for.h>
#include <absolutePoseEstimation/translationAveraging/translationAveraging.h>
#include <mutex>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <boost/graph/connected_components.hpp>

namespace gdr {

    int
    CorrespondenceGraph::refineRelativePose(const VertexCG &vertexToBeTransformed,
                                            const VertexCG &vertexDestination,
                                            SE3 &initEstimationRelPos,
                                            bool &refinementSuccess) {

        MatchableInfo poseToBeTransformed(vertexToBeTransformed.getPathRGBImage(),
                                          vertexToBeTransformed.getPathDImage(),
                                          vertexToBeTransformed.getKeyPoints2D(),
                                          vertexToBeTransformed.getCamera());

        MatchableInfo poseDestination(vertexDestination.getPathRGBImage(),
                                      vertexDestination.getPathDImage(),
                                      vertexDestination.getKeyPoints2D(),
                                      vertexDestination.getCamera());
        refinementSuccess = relativePoseRefiner->refineRelativePose(poseToBeTransformed, poseDestination,
                                                                    initEstimationRelPos);
        assert(refinementSuccess);

        return 0;
    }

    std::vector<std::pair<double, double>> getReprojectionErrorsXY(const Eigen::Matrix4Xd &destinationPoints,
                                                                   const Eigen::Matrix4Xd &transformedPoints,
                                                                   const CameraRGBD &cameraIntrinsics) {
        Eigen::Matrix3d intrinsicsMatrix = cameraIntrinsics.getIntrinsicsMatrix3x3();

        std::vector<std::pair<double, double>> errorsReprojection;
        errorsReprojection.reserve(destinationPoints.cols());
        assert(destinationPoints.cols() == transformedPoints.cols());

        for (int i = 0; i < destinationPoints.cols(); ++i) {
            Eigen::Vector3d homCoordDestination = intrinsicsMatrix * destinationPoints.col(i).topLeftCorner<3, 1>();
            Eigen::Vector3d homCoordTransformed = intrinsicsMatrix * transformedPoints.col(i).topLeftCorner<3, 1>();

            for (int j = 0; j < 2; ++j) {
                homCoordDestination[j] /= homCoordDestination[2];
                homCoordTransformed[j] /= homCoordTransformed[2];
            }
            double errorX = std::abs(homCoordTransformed[0] - homCoordDestination[0]);
            double errorY = std::abs(homCoordTransformed[1] - homCoordDestination[1]);
            errorsReprojection.emplace_back(std::make_pair(errorX, errorY));
        }

        return errorsReprojection;
    }

    /**
     * @param vertexFrom is a vertex number relative transformation from is computed
     * @param vertexInList is a vertex number in vertexFrom's adjacency list (transformation "destination" vertex)
     * @param maxErrorL2 is max L2 error in meters for points to be counted as inliers
     * @param transformation is relative SE3 transformation between poses
     * @param useProjectionError is "true" if reprojection error should be used instead of L2 error
     * @param maxProjectionErrorPixels is max reprojection error value for point pair to be an inlier
     * @param p is parameter for Lp reprojection error metric
     * @returns list of vectors each vector size is 2: for keypoint on the first image and the second
     *      pair is {{obseervingPoseNumber, keyPointLocalIndexOnTheImage}, KeyPointInfo}
     */

    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
    CorrespondenceGraph::findInlierPointCorrespondences(int vertexFrom,
                                                        int vertexInList,
                                                        const SE3 &transformation,
                                                        const ParamsRANSAC &paramsRansac) {

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> correspondencesBetweenTwoImages;
        const auto &match = matches[vertexFrom][vertexInList];
        int minSize = match.matchNumbers.size();

        bool useProjectionError = paramsRansac.getProjectionUsage();
        int p = paramsRansac.getLpMetricParam();


        std::vector<Point3d> toBeTransformedPointsVector;
        std::vector<Point3d> originPointsVector;

        for (int i = 0; i < minSize; ++i) {

            double x_origin, y_origin, z_origin;
            int localIndexOrigin = match.matchNumbers[i].first;
            const auto &siftKeyPointOrigin = verticesOfCorrespondence[vertexFrom].keypoints[localIndexOrigin];
            x_origin = siftKeyPointOrigin.getX();
            y_origin = siftKeyPointOrigin.getY();
            z_origin = verticesOfCorrespondence[vertexFrom].depths[localIndexOrigin];

            originPointsVector.emplace_back(Point3d(x_origin, y_origin, z_origin));
            KeyPointInfo keyPointInfoOrigin(siftKeyPointOrigin, z_origin, vertexFrom);

            std::pair<std::pair<int, int>, KeyPointInfo> infoOriginKeyPoint = {{vertexFrom, localIndexOrigin},
                                                                               KeyPointInfo(siftKeyPointOrigin,
                                                                                            z_origin,
                                                                                            vertexFrom)};

            double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
            int localIndexToBeTransformed = match.matchNumbers[i].second;
            int vertexToBeTransformed = match.frameNumber;
            const auto &siftKeyPointToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].keypoints[localIndexToBeTransformed];
            x_toBeTransformed = siftKeyPointToBeTransformed.getX();
            y_toBeTransformed = siftKeyPointToBeTransformed.getY();
            z_toBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].depths[localIndexToBeTransformed];

            toBeTransformedPointsVector.emplace_back(Point3d(x_toBeTransformed, y_toBeTransformed, z_toBeTransformed));


            std::pair<std::pair<int, int>, KeyPointInfo> infoToBeTransformedKeyPoint = {
                    {vertexToBeTransformed, localIndexToBeTransformed},
                    KeyPointInfo(siftKeyPointToBeTransformed,
                                 z_toBeTransformed,
                                 vertexToBeTransformed)};
            correspondencesBetweenTwoImages.push_back({infoOriginKeyPoint, infoToBeTransformedKeyPoint});

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(originPointsVector.size() == minSize);

        const auto &poseToDestination = verticesOfCorrespondence[match.frameNumber];

        Eigen::Matrix4Xd toBeTransformedPoints = verticesOfCorrespondence[vertexFrom].getCamera().
                getPointCloudXYZ1BeforeProjection(toBeTransformedPointsVector);
        Eigen::Matrix4Xd destinationPoints = poseToDestination.getCamera().getPointCloudXYZ1BeforeProjection(
                originPointsVector);

        Eigen::Matrix4Xd transformedPoints = transformation.getSE3().matrix() * toBeTransformedPoints;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondences;

        if (useProjectionError) {
            std::vector<std::pair<double, double>> errorsReprojection =
                    getReprojectionErrorsXY(destinationPoints,
                                            transformedPoints,
                                            poseToDestination.getCamera());

            assert(errorsReprojection.size() == destinationPoints.cols());

            for (int i = 0; i < errorsReprojection.size(); ++i) {
                Sophus::Vector2d errorReprojectionVector(errorsReprojection[i].first, errorsReprojection[i].second);
                double errorNormLp = 0;
                if (p == 1) {
                    errorNormLp = errorReprojectionVector.lpNorm<1>();
                } else if (p == 2) {
                    errorNormLp = errorReprojectionVector.lpNorm<2>();
                } else {
                    assert(false && "only p=1 and p=2 L_p norms for reprojection error can be used");
                }
                if (errorNormLp < paramsRansac.getMaxProjectionErrorPixels()) {
                    inlierCorrespondences.emplace_back(correspondencesBetweenTwoImages[i]);
                }
            }

        } else {
            Eigen::Matrix4Xd residuals = destinationPoints - transformedPoints;
            for (int i = 0; i < residuals.cols(); ++i) {

                double normResidual = residuals.col(i).norm();
                if (normResidual < paramsRansac.getMax3DError()) {
                    inlierCorrespondences.push_back(correspondencesBetweenTwoImages[i]);
                }
            }
        }

        return inlierCorrespondences;
    }


    int CorrespondenceGraph::findTransformationRtMatrices() {

        std::mutex output;
        tbb::concurrent_vector<tbb::concurrent_vector<RelativeSE3>> transformationMatricesConcurrent(
                verticesOfCorrespondence.size());
        transformationMatricesICP.resize(verticesOfCorrespondence.size());
        transformationMatricesLoRansac.resize(verticesOfCorrespondence.size());

        int matchFromIndex = 0;
        std::mutex indexFromMutex;
        tbb::parallel_for(0, static_cast<int>(matches.size()),
                          [&matchFromIndex, &indexFromMutex, this,
                                  &transformationMatricesConcurrent, &output](int) {
                              int i = -1;
                              {
                                  std::unique_lock<std::mutex> lockCounterFrom(indexFromMutex);
                                  i = matchFromIndex;
                                  assert(matchFromIndex >= 0 && matchFromIndex < verticesOfCorrespondence.size());
                                  ++matchFromIndex;
                              }
                              int matchToIndex = 0;
                              std::mutex indexToMutex;

                              tbb::parallel_for(0, static_cast<int>(matches[i].size()),
                                                [&matchToIndex, &indexToMutex, i,
                                                        &transformationMatricesConcurrent, this,
                                                        &output](int) {

                                                    int jPos = -1;
                                                    {
                                                        std::unique_lock<std::mutex> lockCounterTo(indexToMutex);
                                                        jPos = matchToIndex;
                                                        assert(matchToIndex >= 0 && matchToIndex < matches[i].size());
                                                        ++matchToIndex;
                                                    }
                                                    int j = jPos;
                                                    const auto &match = matches[i][j];
                                                    const auto &frameFromDestination = verticesOfCorrespondence[i];
                                                    const auto &frameToToBeTransformed = verticesOfCorrespondence[match.frameNumber];
                                                    assert(frameToToBeTransformed.getIndex() >
                                                           frameFromDestination.getIndex());
                                                    bool success = true;
                                                    bool successICP = true;
                                                    auto cameraMotion = getTransformationRtMatrixTwoImages(i, j,
                                                                                                           success,
                                                                                                           ParamsRANSAC());

                                                    if (success) {
                                                        int spaceIO = 18;
                                                        {
                                                            std::unique_lock<std::mutex> lockOutput(output);
                                                            std::cout << "success frameFrom -> frameTo: " << std::endl
                                                                      << "             " << frameFromDestination.index
                                                                      << " -> "
                                                                      << frameToToBeTransformed.index << std::endl;
                                                        }

                                                        Eigen::Matrix3d m3d = cameraMotion.block(0, 0, 3, 3);
                                                        Eigen::Quaterniond qRelatived(m3d);
                                                        Sophus::SE3d relativeTransformationSE3 = Sophus::SE3d::fitToSE3(
                                                                cameraMotion);

                                                        // fill info about relative pairwise transformations Rt
                                                        transformationMatricesConcurrent[i].push_back(
                                                                RelativeSE3(
                                                                        relativeTransformationSE3.matrix(),
                                                                        frameFromDestination,
                                                                        frameToToBeTransformed));
                                                        transformationMatricesConcurrent[frameToToBeTransformed.index].push_back(
                                                                RelativeSE3(
                                                                        relativeTransformationSE3.inverse().matrix(),
                                                                        frameToToBeTransformed,
                                                                        frameFromDestination));

                                                    } else {


                                                        std::unique_lock<std::mutex> lockOutput(output);
                                                        std::cout << "                             NOT ___success____ "
                                                                  << frameFromDestination.index
                                                                  << " -> "
                                                                  << frameToToBeTransformed.index << " \tmatches "
                                                                  << match.matchNumbers.size()
                                                                  << std::endl;
                                                    }
                                                });
                          });

        assert(transformationMatricesConcurrent.size() == transformationRtMatrices.size());
        for (int i = 0; i < transformationMatricesConcurrent.size(); ++i) {
            for (const auto &transformation: transformationMatricesConcurrent[i]) {
                transformationRtMatrices[i].push_back(transformation);
            }
        }

        return 0;
    }

    void CorrespondenceGraph::decreaseDensity() {
        for (std::vector<Match> &correspondenceList: matches) {

            std::sort(correspondenceList.begin(), correspondenceList.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.matchNumbers.size() > rhs.matchNumbers.size();
            });

            if (correspondenceList.size() > maxVertexDegree) {
                std::vector<Match> newMatchList(correspondenceList.begin(),
                                                correspondenceList.begin() + maxVertexDegree);
                std::swap(correspondenceList, newMatchList);
            }
        }
    }

    Eigen::Matrix4d
    CorrespondenceGraph::getTransformationRtMatrixTwoImages(int vertexFromDestOrigin,
                                                            int vertexInListToBeTransformedCanBeComputed,
                                                            bool &success,
                                                            const ParamsRANSAC &paramsRansac,
                                                            bool showMatchesOnImages) {
        Eigen::Matrix4d cR_t_umeyama;
        cR_t_umeyama.setIdentity();
        success = true;

        double inlierCoeff = paramsRansac.getInlierCoeff();

        if (inlierCoeff >= 1.0) {
            inlierCoeff = 1.0;
        }
        if (inlierCoeff < 0) {
            success = false;
            return cR_t_umeyama;
        }

        const auto &match = matches[vertexFromDestOrigin][vertexInListToBeTransformedCanBeComputed];
        int minSize = match.matchNumbers.size();
        if (minSize < paramsRansac.getInlierNumber()) {
            success = false;
            return cR_t_umeyama;
        }


        std::vector<Point3d> toBeTransformedPointsVector;
        std::vector<Point3d> originPointsVector;
        int vertexToBeTransformed = match.frameNumber;

        for (int i = 0; i < minSize; ++i) {

            double x_origin, y_origin, z_origin;
            int localIndexOrigin = match.matchNumbers[i].first;
            const auto &siftKeyPointOrigin = verticesOfCorrespondence[vertexFromDestOrigin].keypoints[localIndexOrigin];
            x_origin = siftKeyPointOrigin.getX();
            y_origin = siftKeyPointOrigin.getY();
            z_origin = verticesOfCorrespondence[vertexFromDestOrigin].depths[localIndexOrigin];
            originPointsVector.emplace_back(Point3d(x_origin, y_origin, z_origin));

            double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
            int localIndexToBeTransformed = match.matchNumbers[i].second;

            const auto &siftKeyPointToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].keypoints[localIndexToBeTransformed];
            x_toBeTransformed = siftKeyPointToBeTransformed.getX();
            y_toBeTransformed = siftKeyPointToBeTransformed.getY();
            z_toBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].depths[localIndexToBeTransformed];
            toBeTransformedPointsVector.emplace_back(Point3d(x_toBeTransformed, y_toBeTransformed, z_toBeTransformed));

            std::vector<std::pair<int, int>> points = {{vertexFromDestOrigin,  localIndexOrigin},
                                                       {vertexToBeTransformed, localIndexToBeTransformed}};

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(originPointsVector.size() == minSize);


        Eigen::Matrix4Xd toBeTransformedPoints = verticesOfCorrespondence[vertexFromDestOrigin].getCamera()
                .getPointCloudXYZ1BeforeProjection(toBeTransformedPointsVector);
        Eigen::Matrix4Xd originPoints = verticesOfCorrespondence[match.frameNumber].getCamera().
                getPointCloudXYZ1BeforeProjection(originPointsVector);

        assert(toBeTransformedPoints.cols() == minSize);
        assert(originPoints.cols() == minSize);

        const auto &cameraToBeTransformed = verticesOfCorrespondence[vertexFromDestOrigin].getCamera();
        const auto &cameraDest = verticesOfCorrespondence[match.frameNumber].getCamera();
        std::vector<int> inliersAgain;
        SE3 relativePoseLoRANSAC = relativePoseEstimatorRobust->estimateRelativePose(toBeTransformedPoints,
                                                                                     originPoints,
                                                                                     cameraToBeTransformed,
                                                                                     cameraDest,
                                                                                     paramsRansac,
                                                                                     success,
                                                                                     inliersAgain);
        if (!success) {
            cR_t_umeyama.setIdentity();
            return cR_t_umeyama;
        }

        Eigen::Matrix4d ransacPose = relativePoseLoRANSAC.getSE3().matrix();
        auto inlierMatchesCorrespondingKeypointsLoRansac = findInlierPointCorrespondences(vertexFromDestOrigin,
                                                                                          vertexInListToBeTransformedCanBeComputed,
                                                                                          relativePoseLoRANSAC,
                                                                                          paramsRansac);
        std::cout << "NEW INLIERS " << inliersAgain.size() << std::endl;
        std::cout << "OLD INLIERS == NEW INLIERS " << inlierMatchesCorrespondingKeypointsLoRansac.size() << std::endl;
        assert(inliersAgain.size() == inlierMatchesCorrespondingKeypointsLoRansac.size());
        assert(inliersAgain.size() >= inlierCoeff * toBeTransformedPoints.cols());

        bool successRefine = true;
        SE3 refinedByICPRelativePose = relativePoseLoRANSAC;
        refineRelativePose(verticesOfCorrespondence[vertexToBeTransformed],
                           verticesOfCorrespondence[vertexFromDestOrigin],
                           refinedByICPRelativePose,
                           successRefine);

        transformationMatricesICP[vertexFromDestOrigin].insert(
                std::make_pair(vertexToBeTransformed,
                               RelativeSE3(refinedByICPRelativePose,
                                           verticesOfCorrespondence[vertexFromDestOrigin],
                                           verticesOfCorrespondence[vertexToBeTransformed])));
        transformationMatricesICP[vertexToBeTransformed].insert(
                std::make_pair(vertexFromDestOrigin,
                               RelativeSE3(
                                       refinedByICPRelativePose.inverse(),
                                       verticesOfCorrespondence[vertexToBeTransformed],
                                       verticesOfCorrespondence[vertexFromDestOrigin])));


        auto inlierMatchesCorrespondingKeypointsAfterRefinement =
                findInlierPointCorrespondences(vertexFromDestOrigin,
                                               vertexInListToBeTransformedCanBeComputed,
                                               refinedByICPRelativePose,
                                               paramsRansac);

        ++totalMeausedRelativePoses;
        int ransacInliers = inliersAgain.size();
        int ICPinliers = inlierMatchesCorrespondingKeypointsAfterRefinement.size();
        std::cout << "              ransac got " << ransacInliers << "/" << toBeTransformedPoints.cols()
                  << " vs " << ICPinliers << std::endl;

        if (ransacInliers > ICPinliers) {
            // ICP did not refine the relative pose -- return umeyama result
            cR_t_umeyama = relativePoseLoRANSAC.getSE3().matrix();

        } else {
            cR_t_umeyama = refinedByICPRelativePose.getSE3().matrix();
            // ICP did refine the relative pose -- return ICP inliers TODO
            std::cout << "REFINED________________________________________________" << std::endl;
            ++refinedPoses;
            pairsWhereGotBetterResults[vertexFromDestOrigin].push_back(vertexToBeTransformed);
            pairsWhereGotBetterResults[vertexToBeTransformed].push_back(vertexFromDestOrigin);
            // TODO:

            std::swap(inlierMatchesCorrespondingKeypointsAfterRefinement, inlierMatchesCorrespondingKeypointsLoRansac);
        }


        if (inliersAgain.size() > ransacInliers) {
            std::cout << "BETTER NOW!!!!" << std::endl;
        }


        std::vector<std::pair<int, int>> matchesForVisualization;
        for (const auto &matchPair: inlierMatchesCorrespondingKeypointsLoRansac) {
            tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>> matchesConcurrent;
            for (const auto &matchEntry: matchPair) {
                matchesConcurrent.push_back(matchEntry);
            }
            matchesForVisualization.push_back({matchPair[0].first.second, matchPair[1].first.second});
            inlierCorrespondencesPoints.push_back(matchesConcurrent);
        }
        const auto &poseFrom = verticesOfCorrespondence[vertexFromDestOrigin];
        const auto &poseTo = verticesOfCorrespondence[vertexToBeTransformed];

        if (showMatchesOnImages) {
            ImageDrawer::showKeyPointMatchesTwoImages(poseFrom.getPathRGBImage(),
                                                      poseFrom.getKeyPointInfoAllKeyPoints(),
                                                      poseTo.getPathRGBImage(), poseTo.getKeyPointInfoAllKeyPoints(),
                                                      matchesForVisualization);
        }
        return cR_t_umeyama;
    }

    int CorrespondenceGraph::printRelativePosesFile(const std::string &pathOutRelativePoseFile) {

        std::ofstream file(pathOutRelativePoseFile);

        if (file.is_open()) {
            int numPoses = transformationRtMatrices.size();
            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }
            for (int i = 0; i < transformationRtMatrices.size(); ++i) {
                for (int j = 0; j < transformationRtMatrices[i].size(); ++j) {
                    if (i >= transformationRtMatrices[i][j].getIndexTo()) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = transformationRtMatrices[i][j].getIndexTo();
                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)
                    // (gtsam format)
                    file << "EDGE_SE3:QUAT " << indexFrom << ' ' << indexTo << ' ';
                    auto translationVector = transformationRtMatrices[i][j].getRelativeTranslation();
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &qR = transformationRtMatrices[i][j].getRelativeRotation();

                    file << std::to_string(qR.x()) << ' ' << std::to_string(qR.y()) << ' ' <<
                         std::to_string(qR.z()) << ' '
                         << std::to_string(qR.w()) << noise << '\n';
                }
            }
        }

        return 0;
    }

    keyPointsDepthDescriptor filterKeypointsByKnownDepth(
            const std::pair<std::vector<KeyPoint2D>, std::vector<float>> &keypointAndDescriptor,
            const std::string &pathToDImage) {

        double depthCoefficient = 5000.0;
        std::cout << "start filtering keypoints for image pair " << pathToDImage << std::endl;
        const std::vector<KeyPoint2D> &keypoints = keypointAndDescriptor.first;
        const std::vector<float> &descriptors = keypointAndDescriptor.second;
        std::vector<KeyPoint2D> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        cv::Mat depthImage = cv::imread(pathToDImage, cv::IMREAD_ANYDEPTH);

        for (int i = 0; i < keypoints.size(); ++i) {
            int posInDescriptorVector = 128 * i;
            int maxDepthValue = 65536;
            auto coordY = keypoints[i].getY();
            auto coordX = keypoints[i].getX();

            assert(coordX >= 0 && coordX < depthImage.cols);
            assert(coordY >= 0 && coordY < depthImage.rows);
            int currentKeypointDepth = depthImage.at<ushort>(static_cast<ushort>(coordY),
                                                             static_cast<ushort>(coordX));

            if (currentKeypointDepth > 0) {
                assert(currentKeypointDepth < maxDepthValue);
                depths.push_back(currentKeypointDepth / depthCoefficient);
                keypointsKnownDepth.push_back(keypoints[i]);
                std::vector<float> currentDescriptors;
                for (int descriptorCounter = 0; descriptorCounter < 128; ++descriptorCounter) {
                    descriptorsKnownDepth.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                    currentDescriptors.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                }
            }
        }

        return keyPointsDepthDescriptor(keypointsKnownDepth, descriptorsKnownDepth, depths);

    }

    int CorrespondenceGraph::computeRelativePoses() {

        std::cout << "start computing descriptors" << std::endl;

        std::vector<std::pair<std::vector<KeyPoint2D>, std::vector<float>>>
                keysDescriptorsAll = siftModule->getKeypoints2DDescriptorsAllImages(
                imagesRgb,
                {0});

        verticesOfCorrespondence.reserve(keysDescriptorsAll.size());

        for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {

            keyPointsDepthDescriptor keyPointsDepthDescriptor = filterKeypointsByKnownDepth(
                    keysDescriptorsAll[currentImage], imagesD[currentImage]);
            VertexCG currentVertex(currentImage,
                                   cameraRgbd,
                                   keyPointsDepthDescriptor,
                                   imagesRgb[currentImage],
                                   imagesD[currentImage]);
            verticesOfCorrespondence.push_back(currentVertex);
            assert(verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].depths.size() ==
                   verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].keypoints.size());
        }
        matches = siftModule->findCorrespondences(verticesOfCorrespondence);

        decreaseDensity();
        findTransformationRtMatrices();
        std::string poseFile = relativePose;
        printRelativePosesFile(poseFile);

        return 0;
    }

    CorrespondenceGraph::CorrespondenceGraph(const std::string &newPathToImageDirectoryRGB,
                                             const std::string &newPathToImageDirectoryD,
                                             float fx, float cx, float fy, float cy,
                                             int numOfThreadsCpu) :
            cameraRgbd({fx, cx, fy, cy}),
            pathToImageDirectoryRGB(newPathToImageDirectoryRGB),
            pathToImageDirectoryD(newPathToImageDirectoryD) {

        siftModule = std::make_unique<SiftModuleGPU>();
        relativePoseEstimatorRobust = std::make_unique<EstimatorRobustLoRANSAC>();
        relativePoseRefiner = std::make_unique<ProcessorICP>();
        threadPool = std::make_unique<ThreadPool>(numOfThreadsCpu);
        std::cout << "construct Graph" << std::endl;
        imagesRgb = DirectoryReader::readPathsToImagesFromDirectory(pathToImageDirectoryRGB);
        imagesD = DirectoryReader::readPathsToImagesFromDirectory(pathToImageDirectoryD);
        std::cout << "data have been read" << std::endl;

        std::sort(imagesRgb.begin(), imagesRgb.end());
        std::sort(imagesD.begin(), imagesD.end());


        PRINT_PROGRESS("images rgb vs d: " << imagesRgb.size() << " vs " << imagesD.size());
        assert(imagesRgb.size() == imagesD.size());

        int numberOfPoses = std::min(imagesRgb.size(), imagesD.size());
        pairsWhereGotBetterResults.resize(numberOfPoses);

        transformationRtMatrices = std::vector<std::vector<RelativeSE3>>(imagesD.size());

        PRINT_PROGRESS("Totally read " << imagesRgb.size());
    }

    void CorrespondenceGraph::printConnectionsRelative(std::ostream &os, int space) {

        int counter = 0;
        int counterSquared = 0;
        os << "EDGES of the Correspondence Graph:" << std::endl;
        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            os << std::setw(space / 5) << i << ":";
            counter += transformationRtMatrices[i].size();
            counterSquared += transformationRtMatrices[i].size() * transformationRtMatrices[i].size();
            for (int j = 0; j < transformationRtMatrices[i].size(); ++j) {
                const RelativeSE3 &e = transformationRtMatrices[i][j];
                assert(i == e.getIndexFrom());
                os << std::setw(space / 2) << e.getIndexTo() << ",";
            }
            os << std::endl;
        }
        os << "average number of edges " << counter / transformationRtMatrices.size() << std::endl;

        os << "sq D " << sqrt(counterSquared * 1.0 / transformationRtMatrices.size() -
                              pow(counter * 1.0 / transformationRtMatrices.size(), 2)) << std::endl;

    }


    std::vector<std::vector<int>>
    CorrespondenceGraph::bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const {

        class V {
        };

        class C {
        };

        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;
        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (const auto &pose: verticesOfCorrespondence) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            for (const auto &edge: transformationRtMatrices[i]) {
                assert(i == edge.getIndexFrom());

                if (edge.getIndexFrom() > edge.getIndexTo()) {
                    continue;
                }
                boost::add_edge(verticesBoost[edge.getIndexFrom()], verticesBoost[edge.getIndexTo()], poseGraphForBfs);
            }
        }

        std::vector<int> component(boost::num_vertices(poseGraphForBfs));
        size_t num_components = boost::connected_components(poseGraphForBfs, component.data());

        std::vector<std::vector<int>> components(num_components);

        for (int i = 0; i < component.size(); ++i) {
            components[component[i]].push_back(i);
        }

        componentNumberByPoseIndex = component;
        assert(!componentNumberByPoseIndex.empty());

        for (int componentNum = 0; componentNum < components.size(); ++componentNum) {
            for (const auto &pose: components[componentNum]) {
                assert(componentNumberByPoseIndex[pose] == componentNum);
            }
        }
        return components;
    }

    void CorrespondenceGraph::bfsDrawToFile(const std::string &outFile) const {

        class V {
        };
        class C {
        };
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;

        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (const auto &pose: verticesOfCorrespondence) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            for (const auto &edge: transformationRtMatrices[i]) {
                assert(i == edge.getIndexFrom());

                if (edge.getIndexFrom() > edge.getIndexTo()) {
                    continue;
                }
                boost::add_edge(verticesBoost[edge.getIndexFrom()], verticesBoost[edge.getIndexTo()], poseGraphForBfs);
            }
        }

        if (!outFile.empty()) {
            std::ofstream outf(outFile);
            boost::write_graphviz(outf, poseGraphForBfs);
        }
    }

    std::vector<ConnectedComponentPoseGraph> CorrespondenceGraph::splitGraphToConnectedComponents() const {

        std::vector<int> componentNumberByPose;
        std::vector<std::vector<int>> components = bfsComputeConnectedComponents(componentNumberByPose);

        for (int i = 0; i < components.size(); ++i) {
            std::cout << " component " << std::setw(4) << i << "-th size: " << components[i].size() << " elements: ";

            for (const auto &pose: components[i]) {
                std::cout << pose << ' ';
                assert(componentNumberByPose[pose] == i);
            }
            std::cout << std::endl;
        }
        std::vector<ConnectedComponentPoseGraph> connectedComponents;

        // fill information about poses and change indices inside each components
        // so they are locally sequentially packed [0.. component.size() - 1]

        // components container
        std::vector<std::vector<VertexCG>> connectedComponentsVertices(components.size());

        // relative poses SE3 containers
        // each vector element represents one connected component connections (edges)
        // each vector of vectors is components.size() sized
        // and i-th vector containes edges from i-th pose (local index)
        std::vector<std::vector<std::vector<RelativePoseSE3>>> edgesOfComponentsByComponentsNumber(components.size());

        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            edgesOfComponentsByComponentsNumber[componentNumber].resize(components[componentNumber].size());
        }
        // contains pair {connected component number, "sequentially packed" index inside component}
        // for each pose
        std::vector<std::pair<int, int>> componentNumberAndLocalIndexByPoseGlobalIndex;

        for (int poseGlobalIndex = 0; poseGlobalIndex < verticesOfCorrespondence.size(); ++poseGlobalIndex) {

            VertexCG poseInsideConnectedComponent(verticesOfCorrespondence[poseGlobalIndex]);
            int componentNumber = componentNumberByPose[poseGlobalIndex];
            int localIndexInsideComponent = connectedComponentsVertices[componentNumber].size();
            componentNumberAndLocalIndexByPoseGlobalIndex.emplace_back(
                    std::make_pair(componentNumber, localIndexInsideComponent));
            poseInsideConnectedComponent.setIndex(localIndexInsideComponent);

            connectedComponentsVertices[componentNumber].emplace_back(poseInsideConnectedComponent);
        }

        assert(componentNumberAndLocalIndexByPoseGlobalIndex.size() == verticesOfCorrespondence.size());

        for (int globalPoseIndex = 0; globalPoseIndex < verticesOfCorrespondence.size(); ++globalPoseIndex) {
            const auto &infoAboutpose = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndex];
            std::cout << "  INFO about #global pose " << globalPoseIndex
                      << " in #component " << infoAboutpose.first
                      << " index inside component is: " << infoAboutpose.second << std::endl;
        }
        assert(transformationRtMatrices.size() == verticesOfCorrespondence.size());

        // recompute transformation matrices using new local indices
        for (int indexFrom = 0; indexFrom < transformationRtMatrices.size(); ++indexFrom) {
            for (const auto &transformation: transformationRtMatrices[indexFrom]) {
                assert(transformation.getIndexFrom() == indexFrom);

                int indexTo = transformation.getIndexTo();
                int componentNumberIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].first;
                int localIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].second;

                int componentNumberIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].first;
                int localIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].second;

                assert(componentNumberIndexFrom == componentNumberIndexTo);

                const Sophus::SE3d &relativePoseSE3 = transformation.getRelativePoseSE3();
                RelativePoseSE3 localRelativePoseSE3(localIndexFrom,
                                                     localIndexTo,
                                                     transformation.getRelativePose());
                assert(componentNumberByPose[indexFrom] == componentNumberByPose[indexTo]);
                assert(componentNumberByPose[indexFrom] == componentNumberIndexTo);

                assert(localIndexFrom < edgesOfComponentsByComponentsNumber[componentNumberIndexFrom].size());
                assert(localRelativePoseSE3.getIndexFrom() == localIndexFrom);
                assert(localRelativePoseSE3.getIndexTo() == localIndexTo);
                edgesOfComponentsByComponentsNumber[componentNumberIndexTo][localIndexFrom].emplace_back(
                        localRelativePoseSE3);

            }
        }

        //fill information about observed points with local pose indices
        std::vector<std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>>
                inlierCorrespondencesPointsInsideComponentByComponentNumber(components.size());

        for (const auto &pairOfMatchedKeyPoints: inlierCorrespondencesPoints) {
            assert(pairOfMatchedKeyPoints.size() == 2);
            int globalPoseIndexFirst = pairOfMatchedKeyPoints[0].first.first;
            int globalPoseIndexSecond = pairOfMatchedKeyPoints[1].first.first;

            int localPoseIndexFirst = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexFirst].second;
            int localPoseIndexSecond = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexSecond].second;
            int componentNumber = componentNumberByPose[globalPoseIndexFirst];

            const auto &pairFirst = pairOfMatchedKeyPoints[0];
            const auto &pairSecond = pairOfMatchedKeyPoints[1];
            KeyPointInfo keyPointInfoFirst = pairFirst.second;
            keyPointInfoFirst.setObservingPoseNumber(localPoseIndexFirst);
            KeyPointInfo keyPointInfoSecond = pairSecond.second;
            keyPointInfoSecond.setObservingPoseNumber(localPoseIndexSecond);

            std::pair<std::pair<int, int>, KeyPointInfo> pointInfoLocalFirst = {
                    {localPoseIndexFirst, pairFirst.first.second}, keyPointInfoFirst};
            std::pair<std::pair<int, int>, KeyPointInfo> pointInfoLocalSecond = {
                    {localPoseIndexSecond, pairSecond.first.second}, keyPointInfoSecond};

            assert(componentNumber == componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexSecond].first);

            inlierCorrespondencesPointsInsideComponentByComponentNumber[componentNumber].push_back(
                    {pointInfoLocalFirst, pointInfoLocalSecond});
        }

        std::vector<std::pair<int, int>> componentSizeAndComponentNumber;
        for (int i = 0; i < components.size(); ++i) {
            componentSizeAndComponentNumber.emplace_back(std::make_pair(components[i].size(), i));
        }

        std::stable_sort(componentSizeAndComponentNumber.begin(),
                         componentSizeAndComponentNumber.end(),
                         std::greater<>());
        std::unordered_map<int, int> numberWhenSortedBySizeByGlobalIndex;

        for (int i = 0; i < componentSizeAndComponentNumber.size(); ++i) {
            auto &sizeAndIndex = componentSizeAndComponentNumber[i];
            numberWhenSortedBySizeByGlobalIndex[sizeAndIndex.second] = i;
        }
        for (const auto &indexAndIndexWhenSorted: numberWhenSortedBySizeByGlobalIndex) {
            std::cout << " for pose #indexed " << indexAndIndexWhenSorted.first << " sorted pose is "
                      << indexAndIndexWhenSorted.second << std::endl;
        }
        int addedVertices = 0;
        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            addedVertices += connectedComponentsVertices[componentNumber].size();
            int numberOfComponents = components.size();
            assert(numberOfComponents == connectedComponentsVertices.size());
            assert(numberOfComponents == edgesOfComponentsByComponentsNumber.size());
            assert(numberOfComponents == inlierCorrespondencesPointsInsideComponentByComponentNumber.size());
            int indexWhenSorted = numberWhenSortedBySizeByGlobalIndex[componentNumber];
            std::string namePrefix =
                    "comp_" + std::to_string(numberWhenSortedBySizeByGlobalIndex[componentNumber]) + "_";

            connectedComponents.emplace_back(
                    ConnectedComponentPoseGraph(connectedComponentsVertices[componentNumber],
                                                edgesOfComponentsByComponentsNumber[componentNumber],
                                                cameraRgbd,
                                                inlierCorrespondencesPointsInsideComponentByComponentNumber[componentNumber],
                                                namePrefix + relativePose,
                                                namePrefix + absolutePose,
                                                componentNumber));
        }
        assert(addedVertices == verticesOfCorrespondence.size());

        std::stable_sort(connectedComponents.begin(), connectedComponents.end(),
                         [](const auto &lhs, const auto &rhs) {
                             return lhs.getNumberOfPoses() > rhs.getNumberOfPoses();
                         });

        return connectedComponents;
    }
}
