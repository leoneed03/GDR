//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CorrespondenceGraph.h"
#include "BundleAduster.h"
#include "printer.h"
#include "ICP.h"
#include "pointCloud.h"
#include "RotationOptimizationRobust.h"
#include "ImageDrawer.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <tbb/parallel_for.h>
#include <translationAveraging.h>
#include <mutex>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>

#include <boost/graph/connected_components.hpp>

namespace gdr {


    int
    CorrespondenceGraph::refineRelativePose(const VertexCG &vertexToBeTransformed,
                                            const VertexCG &vertexDestination,
                                            Eigen::Matrix4d &initEstimationRelPos,
                                            bool &success) {
//        VertexCG vertexToBeTransformed = verticesOfCorrespondence[vertexFrom];
        ///wrong vertexNumber -- see struct "Match"
//        VertexCG vertexDestination = verticesOfCorrespondence[vertexTo];
        ProcessorICP::refineRelativePoseICP(vertexToBeTransformed, vertexDestination, initEstimationRelPos);

        return 0;
    }

    std::vector<std::pair<double, double>> getReprojectionErrorsXY(const Eigen::Matrix4Xd& destinationPoints,
                                                     const Eigen::Matrix4Xd& transformedPoints,
                                                     const CameraRGBD& cameraIntrinsics) {
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


    // TODO: calculate inliers using projection error
    /*
     * return list of vectors
     * each vector size is 2: for keypoint on the first image and the second
     * pair is {{obseervingPoseNumber, keyPointLocalIndexOnTheImage}, KeyPointInfo}
     */
    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
    CorrespondenceGraph::findInlierPointCorrespondences(int vertexFrom,
                                                        int vertexInList,
                                                        double maxErrorL2,
                                                        Eigen::Matrix4d &transformation,
                                                        bool useProjectionError,
                                                        double maxProjectionErrorPixels) {
        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> correspondencesBetweenTwoImages;
        const auto &match = matches[vertexFrom][vertexInList];
        int minSize = match.matchNumbers.size();

        std::vector<std::vector<double>> toBeTransformedPointsVector;
        std::vector<std::vector<double>> originPointsVector;

        for (int i = 0; i < minSize; ++i) {

            double x_origin, y_origin, z_origin;
            int localIndexOrigin = match.matchNumbers[i].first;
            const auto &siftKeyPointOrigin = verticesOfCorrespondence[vertexFrom].keypoints[localIndexOrigin];
            x_origin = siftKeyPointOrigin.x;
            y_origin = siftKeyPointOrigin.y;
            z_origin = verticesOfCorrespondence[vertexFrom].depths[localIndexOrigin];

            originPointsVector.push_back({x_origin, y_origin, z_origin, 1});
            KeyPointInfo keyPointInfoOrigin(siftKeyPointOrigin, z_origin, vertexFrom);


//            std::pair<std::pair<int, int>, KeyPointInfo> infoOriginKeyPoint = std::make_pair(std::make_pair(vertexFrom, localIndexOrigin), keyPointInfoOrigin);
            std::pair<std::pair<int, int>, KeyPointInfo> infoOriginKeyPoint = {{vertexFrom, localIndexOrigin},
                                                                               KeyPointInfo(siftKeyPointOrigin,
                                                                                            z_origin,
                                                                                            vertexFrom)};

            double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
            int localIndexToBeTransformed = match.matchNumbers[i].second;
            int vertexToBeTransformed = match.frameNumber;
            const auto &siftKeyPointToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].keypoints[localIndexToBeTransformed];
            x_toBeTransformed = siftKeyPointToBeTransformed.x;
            y_toBeTransformed = siftKeyPointToBeTransformed.y;
            z_toBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].depths[localIndexToBeTransformed];

            toBeTransformedPointsVector.push_back({x_toBeTransformed, y_toBeTransformed, z_toBeTransformed, 1});


            std::pair<std::pair<int, int>, KeyPointInfo> infoToBeTransformedKeyPoint = {
                    {vertexToBeTransformed, localIndexToBeTransformed},
                    KeyPointInfo(siftKeyPointToBeTransformed,
                                 z_toBeTransformed,
                                 vertexToBeTransformed)};
            correspondencesBetweenTwoImages.push_back({infoOriginKeyPoint, infoToBeTransformedKeyPoint});

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(originPointsVector.size() == minSize);

        const auto& poseToDestination = verticesOfCorrespondence[match.frameNumber];

        Eigen::Matrix4Xd toBeTransformedPoints = getPointCloudBeforeProjection(toBeTransformedPointsVector,
                                                                               verticesOfCorrespondence[vertexFrom].cameraRgbd);
        Eigen::Matrix4Xd destinationPoints = getPointCloudBeforeProjection(originPointsVector,
                                                                      poseToDestination.cameraRgbd);

        Eigen::Matrix4Xd transformedPoints = transformation * toBeTransformedPoints;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondences;

        if (useProjectionError) {
            std::vector<std::pair<double, double>> errorsReprojection =
                    getReprojectionErrorsXY(destinationPoints,
                                            transformedPoints,
                                            poseToDestination.getCamera());

            assert(errorsReprojection.size() == destinationPoints.cols());

            for (int i = 0; i < errorsReprojection.size(); ++i) {
                if (std::max(errorsReprojection[i].first, errorsReprojection[i].second) < maxProjectionErrorPixels) {
                    inlierCorrespondences.emplace_back(correspondencesBetweenTwoImages[i]);
                }
            }

        } else {
            Eigen::Matrix4Xd residuals = destinationPoints - transformedPoints;
            for (int i = 0; i < residuals.cols(); ++i) {

                double normResidual = residuals.col(i).norm();
                if (normResidual < maxErrorL2) {
                    inlierCorrespondences.push_back(correspondencesBetweenTwoImages[i]);
                }

            }
        }

        return inlierCorrespondences;
    }


    int CorrespondenceGraph::findTransformationRtMatrices() {

        std::mutex output;
        tbb::concurrent_vector<tbb::concurrent_vector<transformationRtMatrix>> transformationMatricesConcurrent(
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
                                                                                                           success);

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
                                                                transformationRtMatrix(
                                                                        relativeTransformationSE3.matrix(),
                                                                        frameFromDestination,
                                                                        frameToToBeTransformed));
                                                        transformationMatricesConcurrent[frameToToBeTransformed.index].push_back(
                                                                transformationRtMatrix(
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
                                                            bool useProjection,
                                                            double inlierCoeff,
                                                            double maxProjectionErrorPixels,
                                                            bool showMatchesOnImages) {
        Eigen::Matrix4d cR_t_umeyama;
        cR_t_umeyama.setIdentity();
        success = true;

        if (inlierCoeff >= 1.0) {
            inlierCoeff = 1.0;
        }
        if (inlierCoeff < 0) {
            success = false;
            return cR_t_umeyama;
        }

        const auto &match = matches[vertexFromDestOrigin][vertexInListToBeTransformedCanBeComputed];
        int minSize = match.matchNumbers.size();
        if (minSize < minNumberOfMatches) {
            success = false;
            return cR_t_umeyama;
        }


        std::vector<std::vector<double>> toBeTransformedPointsVector;
        std::vector<std::vector<double>> originPointsVector;
        int vertexToBeTransformed = match.frameNumber;

        for (int i = 0; i < minSize; ++i) {

            double x_origin, y_origin, z_origin;
            int localIndexOrigin = match.matchNumbers[i].first;
            const auto &siftKeyPointOrigin = verticesOfCorrespondence[vertexFromDestOrigin].keypoints[localIndexOrigin];
            x_origin = siftKeyPointOrigin.x;
            y_origin = siftKeyPointOrigin.y;
            z_origin = verticesOfCorrespondence[vertexFromDestOrigin].depths[localIndexOrigin];
            originPointsVector.push_back({x_origin, y_origin, z_origin, 1});

            double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
            int localIndexToBeTransformed = match.matchNumbers[i].second;

            const auto &siftKeyPointToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].keypoints[localIndexToBeTransformed];
            x_toBeTransformed = siftKeyPointToBeTransformed.x;
            y_toBeTransformed = siftKeyPointToBeTransformed.y;
            z_toBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].depths[localIndexToBeTransformed];
            toBeTransformedPointsVector.push_back({x_toBeTransformed, y_toBeTransformed, z_toBeTransformed, 1});

            std::vector<std::pair<int, int>> points = {{vertexFromDestOrigin,  localIndexOrigin},
                                                       {vertexToBeTransformed, localIndexToBeTransformed}};

        }
        assert(toBeTransformedPointsVector.size() == minSize);
        assert(originPointsVector.size() == minSize);


        Eigen::Matrix4Xd toBeTransformedPoints = getPointCloudBeforeProjection(toBeTransformedPointsVector,
                                                                               verticesOfCorrespondence[vertexFromDestOrigin].cameraRgbd);
        Eigen::Matrix4Xd originPoints = getPointCloudBeforeProjection(originPointsVector,
                                                                      verticesOfCorrespondence[match.frameNumber].cameraRgbd);
        assert(toBeTransformedPoints.cols() == minSize);
        assert(originPoints.cols() == minSize);

        if (useProjection) {
            std::cout << "use projection x,y error?! Are you shure?" << std::endl;
            cR_t_umeyama = getTransformationMatrixUmeyamaLoRANSACProjectiveError(toBeTransformedPoints,
                                                                                 originPoints,
                                                                                 verticesOfCorrespondence[vertexToBeTransformed].getCamera().getIntrinsicsMatrix3x3(),
                                                                                 verticesOfCorrespondence[vertexFromDestOrigin].getCamera().getIntrinsicsMatrix3x3(),
                                                                                 numIterations,
                                                                                 minSize,
                                                                                 inlierCoeff,
                                                                                 success,
                                                                                 maxProjectionErrorPixels);
        } else {
            cR_t_umeyama = getTransformationMatrixUmeyamaLoRANSAC(toBeTransformedPoints,
                                                                  originPoints,
                                                                  numIterations,
                                                                  minSize,
                                                                  inlierCoeff,
                                                                  success,
                                                                  neighbourhoodRadius);
        }
        if (!success) {
            cR_t_umeyama.setIdentity();
            return cR_t_umeyama;
        }

        // look for inliers after umeyama
        Sophus::SE3d RtbeforeRefinement = Sophus::SE3d::fitToSE3(cR_t_umeyama);
        transformationMatricesLoRansac[vertexFromDestOrigin].insert(std::make_pair(vertexToBeTransformed,
                                                                                   transformationRtMatrix(
                                                                                           RtbeforeRefinement,
                                                                                           verticesOfCorrespondence[vertexFromDestOrigin],
                                                                                           verticesOfCorrespondence[vertexToBeTransformed])));
        transformationMatricesLoRansac[vertexToBeTransformed].insert(std::make_pair(vertexFromDestOrigin,
                                                                                    transformationRtMatrix(
                                                                                            RtbeforeRefinement.inverse(),
                                                                                            verticesOfCorrespondence[vertexToBeTransformed],
                                                                                            verticesOfCorrespondence[vertexFromDestOrigin])));

        auto inlierMatchesCorrespondingKeypointsLoRansac = findInlierPointCorrespondences(vertexFromDestOrigin,
                                                                                          vertexInListToBeTransformedCanBeComputed,
                                                                                          neighbourhoodRadius,
                                                                                          cR_t_umeyama,
                                                                                          useProjection,
                                                                                          maxProjectionErrorPixels);

        bool successRefine = true;
        refineRelativePose(verticesOfCorrespondence[vertexToBeTransformed],
                           verticesOfCorrespondence[vertexFromDestOrigin], cR_t_umeyama, successRefine);

        Sophus::SE3d RtafterRefinement = Sophus::SE3d::fitToSE3(cR_t_umeyama);
        transformationMatricesICP[vertexFromDestOrigin].insert(std::make_pair(vertexToBeTransformed,
                                                                              transformationRtMatrix(RtafterRefinement,
                                                                                                     verticesOfCorrespondence[vertexFromDestOrigin],
                                                                                                     verticesOfCorrespondence[vertexToBeTransformed])));
        transformationMatricesICP[vertexToBeTransformed].insert(std::make_pair(vertexFromDestOrigin,
                                                                               transformationRtMatrix(
                                                                                       RtafterRefinement.inverse(),
                                                                                       verticesOfCorrespondence[vertexToBeTransformed],
                                                                                       verticesOfCorrespondence[vertexFromDestOrigin])));


        auto inlierMatchesCorrespondingKeypointsAfterRefinement = findInlierPointCorrespondences(vertexFromDestOrigin,
                                                                                                 vertexInListToBeTransformedCanBeComputed,
                                                                                                 neighbourhoodRadius,
                                                                                                 cR_t_umeyama,
                                                                                                 useProjection,
                                                                                                 maxProjectionErrorPixels);

        ++totalMeausedRelativePoses;
        int ransacInliers = inlierMatchesCorrespondingKeypointsLoRansac.size();
        int ICPinliers = inlierMatchesCorrespondingKeypointsAfterRefinement.size();
        std::cout << "              " << "ransac got " << ransacInliers << "/" << toBeTransformedPoints.cols()
                  << " vs " << ICPinliers << std::endl;


        // Show Rt results

//        const VertexCG &frameToToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed];
//        const VertexCG &frameFromDestination = verticesOfCorrespondence[vertexFromDestOrigin];
//
//        cv::Mat transformedCloudProjectedToDestination = getProjectedPointCloud(
//                frameToToBeTransformed.getPathDImage(), RtbeforeRefinement.matrix(),
//                frameToToBeTransformed.getCamera());
//        cv::Mat imageFromDest = cv::imread(frameFromDestination.getPathDImage(), cv::IMREAD_ANYDEPTH);
//        cv::Mat imageToToBeTransformed = cv::imread(frameToToBeTransformed.getPathDImage(),
//                                                    cv::IMREAD_COLOR);
//        cv::Mat transformedCloudProjectedToDestinationICP = getProjectedPointCloud(
//                frameToToBeTransformed.getPathDImage(), cR_t_umeyama, frameToToBeTransformed.getCamera(), true);
//        cv::imshow("From (Dest)", imageFromDest);
//        cv::imshow("To (to be transformed)", imageToToBeTransformed);
//        cv::imshow("To projected -> From LoRansac Umeyama", transformedCloudProjectedToDestination);
//
//        cv::imshow("To projected -> From ICP", transformedCloudProjectedToDestinationICP);
//        cv::waitKey(0);
//        cv::destroyAllWindows();

        if (ransacInliers > ICPinliers) {
            // ICP did not refine the relative pose -- return umeyama result
            cR_t_umeyama = RtbeforeRefinement.matrix();

        } else {
            // ICP did refine the relative pose -- return ICP inliers TODO
            std::cout << "REFINED________________________________________________" << std::endl;
            ++refinedPoses;
            pairsWhereGotBetterResults[vertexFromDestOrigin].push_back(vertexToBeTransformed);
            pairsWhereGotBetterResults[vertexToBeTransformed].push_back(vertexFromDestOrigin);
//            std::swap(inlierMatchesCorrespondingKeypointsAfterRefinement, inlierMatchesCorrespondingKeypointsLoRansac);
        }


        std::vector<std::pair<int, int>> matchesForVisualization;
        for (const auto &matchPair: inlierMatchesCorrespondingKeypointsLoRansac) {
            tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>> matchesConcurrent;
            for (const auto &matchEntry: matchPair) {
                matchesConcurrent.push_back(matchEntry);
            }
            matchesForVisualization.push_back({matchPair[0].first.second, matchPair[1].first.second});
            // TODO: split between connected components
            inlierCorrespondencesPoints.push_back(matchesConcurrent);
        }
        const auto& poseFrom = verticesOfCorrespondence[vertexFromDestOrigin];
        const auto& poseTo = verticesOfCorrespondence[vertexToBeTransformed];
        // TODO: visualize matches

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
                    if (i >= transformationRtMatrices[i][j].vertexTo.index) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = transformationRtMatrices[i][j].vertexTo.index;
                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)(gtsam format) TODO: actually seems like NOT
                    file << "EDGE_SE3:QUAT " << indexFrom << ' ' << indexTo << ' ';
                    auto translationVector = transformationRtMatrices[i][j].t;
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &R = transformationRtMatrices[i][j].R;

                    Eigen::Quaterniond qR(R);
                    std::vector<double> vectorDataRotations = {qR.x(), qR.y(), qR.z(), qR.w()};
                    file << std::to_string(qR.x()) << ' ' << std::to_string(qR.y()) << ' ' <<
                         std::to_string(qR.z()) << ' '
                         << std::to_string(qR.w()) << noise << '\n';
                }
            }
        } else {
            return ERROR_OPENING_FILE_WRITE;
        }

        return 0;
    }

    std::vector<Eigen::Quaterniond> CorrespondenceGraph::performRotationAveraging() {
        PRINT_PROGRESS("first print successfull");

        std::vector<Eigen::Quaterniond> absoluteRotationsQuats = rotationAverager::shanonAveraging(relativePose,
                                                                                                   absolutePose);

        PRINT_PROGRESS("read quaternions successfull");

        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            verticesOfCorrespondence[i].setRotation(absoluteRotationsQuats[i]);
        }

        PRINT_PROGRESS("setting Rotations in vertices successfull");
        return absoluteRotationsQuats;
    }

    keyPointsDepthDescriptor filterKeypointsByKnownDepth(
            const std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> &keypointAndDescriptor,
            const std::string &pathToDImage) {

        double depthCoefficient = 5000.0;
        std::cout << "start filtering keypoints for image pair " << pathToDImage << std::endl;
        const std::vector<SiftGPU::SiftKeypoint> &keypoints = keypointAndDescriptor.first;
        const std::vector<float> &descriptors = keypointAndDescriptor.second;
        std::vector<SiftGPU::SiftKeypoint> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        cv::Mat depthImage = cv::imread(pathToDImage, cv::IMREAD_ANYDEPTH);

        for (int i = 0; i < keypoints.size(); ++i) {
            int posInDescriptorVector = 128 * i;
            int maxDepthValue = 65536;
            auto coordY = keypoints[i].y;
            auto coordX = keypoints[i].x;
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

        return {keypointsKnownDepth, descriptorsKnownDepth, depths};

    }

    int CorrespondenceGraph::computeRelativePoses() {

        std::cout << "start computing descriptors" << std::endl;

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
                keysDescriptorsAll = siftModule.getKeypointsDescriptorsAllImages(readRgbData(pathToImageDirectoryRGB),
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

        {
            std::vector<VertexCG *> posesForCloudProjector;
            posesForCloudProjector.reserve(verticesOfCorrespondence.size());

            for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
                posesForCloudProjector.push_back(&verticesOfCorrespondence[i]);
            }
            assert(posesForCloudProjector.size() == verticesOfCorrespondence.size());
            cloudProjector.setPoses(posesForCloudProjector);
            pointMatcher.setNumberOfPoses(verticesOfCorrespondence.size());
        }
        auto matchedPairs = siftModule.findCorrespondences(verticesOfCorrespondence);
        matches = std::vector<std::vector<Match>>(matchedPairs.size());
        for (int i = 0; i < matchedPairs.size(); ++i) {
            for (const auto &matchPair: matchedPairs[i]) {
                matches[i].push_back(matchPair);
            }
        }
        decreaseDensity();
        findTransformationRtMatrices();
        // TODO: split to disjoint connected components
        computePointClasses();
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

        threadPool = std::make_unique<ThreadPool>(numOfThreadsCpu);
        std::cout << "construct Graph" << std::endl;
        imagesRgb = readRgbData(pathToImageDirectoryRGB);
        imagesD = readRgbData(pathToImageDirectoryD);
        std::cout << "data have been read" << std::endl;

        std::sort(imagesRgb.begin(), imagesRgb.end());
        std::sort(imagesD.begin(), imagesD.end());


        PRINT_PROGRESS("images rgb vs d: " << imagesRgb.size() << " vs " << imagesD.size());
        assert(imagesRgb.size() == imagesD.size());

        int numberOfPoses = std::min(imagesRgb.size(), imagesD.size());
        pairsWhereGotBetterResults.resize(numberOfPoses);

        transformationRtMatrices = std::vector<std::vector<transformationRtMatrix>>(imagesD.size());

        PRINT_PROGRESS("Totally read " << imagesRgb.size());
    }

    int CorrespondenceGraph::printAbsolutePoses(std::ostream &os, int space) {
        os << "======================NOW 4*4 Matrices of absolute positions=======================\n" << std::endl;

        os << "======================++++++++++++++++=======================\n" << std::endl;
        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            os << "Pose number: " << i << std::endl;
            os << verticesOfCorrespondence[i].absoluteRotationTranslation;
            os << "\n_________________________________________________________________\n";
        }
        return 0;
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
                const transformationRtMatrix &e = transformationRtMatrices[i][j];
                assert(i == e.vertexFrom.index);
                os << std::setw(space / 2) << e.vertexTo.index << ",";
            }
            os << std::endl;
        }
        os << "average number of edges " << counter / transformationRtMatrices.size() << std::endl;

        os << "sq D " << sqrt(counterSquared * 1.0 / transformationRtMatrices.size() -
                              pow(counter * 1.0 / transformationRtMatrices.size(), 2)) << std::endl;

    }

    std::vector<int> CorrespondenceGraph::bfs(int currentVertex,
                                              bool &isConnected,
                                              std::vector<std::vector<int>> &connectivityComponents) {
        std::vector<bool> visited(verticesOfCorrespondence.size(), false);
        std::vector<int> preds(verticesOfCorrespondence.size(), -1);
        std::queue<int> queueVertices;
        queueVertices.push(currentVertex);
        assert(verticesOfCorrespondence.size() == transformationRtMatrices.size());

        while (!queueVertices.empty()) {
            int vertex = queueVertices.front();
            std::cout << "bfs entered vertex " << vertex << std::endl;
            queueVertices.pop();
            assert(vertex < visited.size() && vertex >= 0);
            visited[vertex] = true;

            for (int i = 0; i < transformationRtMatrices[vertex].size(); ++i) {
                int to = transformationRtMatrices[vertex][i].vertexTo.index;
                if (!visited[to]) {
                    queueVertices.push(to);
                    visited[to] = true;
                    assert(preds[to] == -1);
                    preds[to] = vertex;

                    ///// get absolute Rotation (R) and Translation (t) with given predecessor Vertex and Relative R & t
//                    const Eigen::Matrix4d &predAbsoluteRt = verticesOfCorrespondence[vertex].absoluteRotationTranslation;
//                    Eigen::Matrix3d predR = predAbsoluteRt.block(0, 0, 3, 3);
//                    Eigen::Vector3d predT = predAbsoluteRt.block(0, 3, 3, 1);
//
//                    const Eigen::Matrix4d &relativeRt = transformationRtMatrices[vertex][i].innerTranformationRtMatrix;
//                    Eigen::Vector3d relT = relativeRt.block(0, 3, 3, 1);
//
//                    Eigen::Matrix4d &newAbsoluteRt = verticesOfCorrespondence[to].absoluteRotationTranslation;
//                    Eigen::Vector3d newAbsoluteT = predR * relT + predT;
//
//                    newAbsoluteRt.block(0, 3, 3, 1) = newAbsoluteT;

                }
            }
        }

        isConnected = true;
        for (int i = 0; i < visited.size(); ++i) {
            if (!visited[i]) {
                isConnected = false;
                return preds;
            }
        }
        return preds;
    }

    /* each inner vector contains matches (normally 2) between keypoint:
     * {[pose number, local point index], keyPointInfo}
     */

    void CorrespondenceGraph::computePointClasses() {
        computePointClasses(inlierCorrespondencesPoints);
    }

    void CorrespondenceGraph::computePointClasses(
            const tbb::concurrent_vector<tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
            &matchesBetweenPoints) {


        for (const tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>>
                    &vectorOfMatches: matchesBetweenPoints) {

            std::vector<std::pair<int, int>> poseAndLocalIndices;
            for (const std::pair<std::pair<int, int>, KeyPointInfo> &fullPointInfo: vectorOfMatches) {
                poseAndLocalIndices.push_back(fullPointInfo.first);
            }
            pointMatcher.insertPointsWithNewClasses(poseAndLocalIndices);
        }


        // unordered map's Key is local index
        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPoseNumAndLocalInd(
                pointMatcher.getNumberOfPoses());


        for (const tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>> &vectorOfMatches: matchesBetweenPoints) {

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

        auto pointClasses = pointMatcher.assignPointClasses();

        for (int pointIncrementor = 0; pointIncrementor < pointClasses.size(); ++pointIncrementor) {
            int pointClassNumber = pointClasses[pointIncrementor];
            std::pair<int, int> poseNumberAndLocalIndex = pointMatcher.getPoseNumberAndLocalIndex(pointIncrementor);
            std::vector<KeyPointInfo> keyPointInfo;
            keyPointInfo.push_back(
                    keyPointInfoByPoseNumAndLocalInd[poseNumberAndLocalIndex.first][poseNumberAndLocalIndex.second]);
            cloudProjector.addPoint(pointClassNumber, keyPointInfo);
        }


    }

    const CloudProjector &CorrespondenceGraph::getCloudProjector() const {
        return cloudProjector;
    }

    std::vector<Eigen::Quaterniond> CorrespondenceGraph::optimizeRotationsRobust() {

        std::vector<Rotation3d> shonanOptimizedAbsolutePoses;

        for (const auto &vertexPose: verticesOfCorrespondence) {
            shonanOptimizedAbsolutePoses.push_back(Rotation3d(vertexPose.getRotationQuat()));
        }

        assert(shonanOptimizedAbsolutePoses.size() == verticesOfCorrespondence.size());


        std::vector<rotationMeasurement> relativeRotationsAfterICP;

        assert(verticesOfCorrespondence.size() == transformationRtMatrices.size());
        for (int indexFrom = 0; indexFrom < verticesOfCorrespondence.size(); ++indexFrom) {
            for (const auto &knownRelativePose: transformationRtMatrices[indexFrom]) {
                assert(indexFrom == knownRelativePose.getIndexFrom());
                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {

                    relativeRotationsAfterICP.push_back(
                            rotationMeasurement(knownRelativePose.getRelativeRotation(),
                                                knownRelativePose.getIndexFrom(),
                                                knownRelativePose.getIndexTo()));
                }
            }
        }

        RotationOptimizer rotationOptimizer(shonanOptimizedAbsolutePoses, relativeRotationsAfterICP);
        std::vector<Eigen::Quaterniond> optimizedPosesRobust = rotationOptimizer.getOptimizedOrientation();

        assert(verticesOfCorrespondence.size() == optimizedPosesRobust.size());
        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            verticesOfCorrespondence[i].setRotation(optimizedPosesRobust[i]);
        }

        return optimizedPosesRobust;
    }

    std::vector<Eigen::Matrix4d> CorrespondenceGraph::getAbsolutePosesEigenMatrix4d() const {
        std::vector<Eigen::Matrix4d> poses;

        for (const auto &pose: verticesOfCorrespondence) {
            poses.push_back(pose.getEigenMatrixAbsolutePose4d());
        }

        return poses;
    }

    std::vector<Eigen::Vector3d> CorrespondenceGraph::optimizeAbsoluteTranslations(int indexFixedToZero) {

        std::vector<translationMeasurement> relativeTranslations;
        std::vector<Eigen::Matrix4d> absolutePoses = getAbsolutePosesEigenMatrix4d();

        for (int indexFrom = 0; indexFrom < verticesOfCorrespondence.size(); ++indexFrom) {
            for (const auto &knownRelativePose: transformationRtMatrices[indexFrom]) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

                if (knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo()) {
                    relativeTranslations.push_back(
                            translationMeasurement(knownRelativePose.getRelativeTranslation(),
                                                   knownRelativePose.getIndexFrom(),
                                                   knownRelativePose.getIndexTo()));
                }
            }
        }

        std::vector<Eigen::Vector3d> optimizedAbsoluteTranslationsIRLS = gdr::translationAverager::recoverTranslations(
                relativeTranslations,
                absolutePoses).toVectorOfVectors();


        bool successIRLS = true;


        // Now run IRLS with PCG answer as init solution

        optimizedAbsoluteTranslationsIRLS = gdr::translationAverager::recoverTranslationsIRLS(
                relativeTranslations,
                absolutePoses,
                optimizedAbsoluteTranslationsIRLS,
                successIRLS).toVectorOfVectors();


        Eigen::Vector3d zeroTranslation = optimizedAbsoluteTranslationsIRLS[indexFixedToZero];
        for (auto &translation: optimizedAbsoluteTranslationsIRLS) {
            translation -= zeroTranslation;
        }


        assert(verticesOfCorrespondence.size() == optimizedAbsoluteTranslationsIRLS.size());
        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            verticesOfCorrespondence[i].setTranslation(optimizedAbsoluteTranslationsIRLS[i]);
        }
        return optimizedAbsoluteTranslationsIRLS;
    }


    std::vector<Sophus::SE3d> CorrespondenceGraph::performBundleAdjustment(int indexFixedToZero) {
        std::vector<Point3d> observedPoints = cloudProjector.setComputedPointsGlobalCoordinates();
        std::cout << "ready " << std::endl;
        std::vector<std::pair<Sophus::SE3d, CameraRGBD>> posesAndCameraParams;
        for (const auto &vertexPose: verticesOfCorrespondence) {
            posesAndCameraParams.push_back({vertexPose.absolutePose, cameraRgbd});
        }
        std::cout << "BA" << std::endl;

        //visualize points and matches
//        cloudProjector.showPointsProjection(observedPoints);



        BundleAdjuster bundleAdjuster(observedPoints, posesAndCameraParams,
                                      cloudProjector.getKeyPointInfoByPoseNumberAndPointClass());

        std::vector<Sophus::SE3d> posesOptimized = bundleAdjuster.optimizePointsAndPoses(indexFixedToZero);

        assert(posesOptimized.size() == verticesOfCorrespondence.size());

        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            auto &vertexPose = verticesOfCorrespondence[i];
            vertexPose.setRotationTranslation(posesOptimized[i]);
        }


        // visualize point correspondences:
//        cloudProjector.showPointsProjection(bundleAdjuster.getPointsGlobalCoordinatesOptimized());
        return posesOptimized;
    }

    std::vector<Sophus::SE3d> CorrespondenceGraph::performBundleAdjustmentUsingDepth(int indexFixedToZero) {
        std::vector<Point3d> observedPoints = cloudProjector.setComputedPointsGlobalCoordinates();
        std::cout << "ready " << std::endl;
        std::vector<std::pair<Sophus::SE3d, CameraRGBD>> posesAndCameraParams;
        for (const auto &vertexPose: verticesOfCorrespondence) {
            posesAndCameraParams.push_back({vertexPose.absolutePose, cameraRgbd});
        }
        std::cout << "BA depth create BundleAdjuster" << std::endl;

        //vizualize points and matches
//        cloudProjector.showPointsProjection(observedPoints);



        BundleAdjuster bundleAdjuster(observedPoints, posesAndCameraParams,
                                      cloudProjector.getKeyPointInfoByPoseNumberAndPointClass());

        std::vector<Sophus::SE3d> posesOptimized = bundleAdjuster.optimizePointsAndPosesUsingDepthInfo(
                indexFixedToZero);

        assert(posesOptimized.size() == verticesOfCorrespondence.size());

        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            auto &vertexPose = verticesOfCorrespondence[i];
            vertexPose.setRotationTranslation(posesOptimized[i]);
        }


        // visualize point correspondences:
//        cloudProjector.showPointsProjection(bundleAdjuster.getPointsGlobalCoordinatesOptimized());
        return posesOptimized;
    }

    class V {
    };

    class C {
    };


    std::vector<std::vector<int>>
    CorrespondenceGraph::bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const {
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

    std::vector<std::vector<int>> CorrespondenceGraph::bfsDrawToFile(const std::string &outFile) const {

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


        if (!outFile.empty()) {
            std::ofstream outf(outFile);
            boost::write_graphviz(outf, poseGraphForBfs);
        }

        return components;
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
                                                     relativePoseSE3);
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
