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

#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <translationAveraging.h>

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

    std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
    CorrespondenceGraph::findInlierPointCorrespondences(int vertexFrom,
                                                        int vertexInList,
                                                        double maxErrorL2,
                                                        Eigen::Matrix4d &transformation,
                                                        bool isICP) {
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


        Eigen::Matrix4Xd toBeTransformedPoints = getPointCloudBeforeProjection(toBeTransformedPointsVector,
                                                                               verticesOfCorrespondence[vertexFrom].cameraRgbd);
        Eigen::Matrix4Xd originPoints = getPointCloudBeforeProjection(originPointsVector,
                                                                      verticesOfCorrespondence[match.frameNumber].cameraRgbd);

        Eigen::Matrix4Xd residuals = originPoints - transformation * toBeTransformedPoints;

        std::vector<bool> inliersCorrespondencesKeyPoints(correspondencesBetweenTwoImages.size(), true);
        assert(inliersCorrespondencesKeyPoints.size() == minSize);
        assert(inliersCorrespondencesKeyPoints.size() == residuals.cols());


        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondences;

        for (int i = 0; i < residuals.cols(); ++i) {
            double normResidual = residuals.col(i).norm();
            if (normResidual < maxErrorL2) {
                inlierCorrespondences.push_back(correspondencesBetweenTwoImages[i]);
            }
        }

        return inlierCorrespondences;
    }


    int CorrespondenceGraph::findTransformationRtMatrices() {

        for (int i = 0; i < matches.size(); ++i) {
            for (int j = 0; j < matches[i].size(); ++j) {

                const auto &match = matches[i][j];
                const auto &frameFromDestination = verticesOfCorrespondence[i];
                const auto &frameToToBeTransformed = verticesOfCorrespondence[match.frameNumber];
                PRINT_PROGRESS("check this " << frameFrom.index << " -> " << frameTo.index);
                assert(frameToToBeTransformed.getIndex() > frameFromDestination.getIndex());
                bool success = true;
                bool successICP = true;
                auto cameraMotion = getTransformationRtMatrixTwoImages(i, j, success);


                PRINT_PROGRESS(
                        "out of Transformation calculation" << std::endl
                                                            << frameFrom.index << " -> " << frameTo.index);

                if (success) {
                    int spaceIO = 18;
                    std::cout << "success frameFrom -> frameTo" << frameFromDestination.index << " -> "
                              << frameToToBeTransformed.index << std::endl;

                    Eigen::Matrix3d m3d = cameraMotion.block(0, 0, 3, 3);
                    Eigen::Quaterniond qRelatived(m3d);

                    PRINT_PROGRESS(std::setw(2 * spaceIO) << qRelatived.x() << std::setw(2 * spaceIO) << qRelatived.y()
                                                          << std::setw(2 * spaceIO) << qRelatived.z()
                                                          << std::setw(2 * spaceIO) << qRelatived.w());

                    Sophus::SE3d relativeTransformationSE3 = Sophus::SE3d::fitToSE3(cameraMotion);

                    // fill info about relative pairwise transformations Rt
                    transformationRtMatrices[i].push_back(
                            transformationRtMatrix(relativeTransformationSE3.matrix(), frameFromDestination,
                                                   frameToToBeTransformed));
                    transformationRtMatrices[frameToToBeTransformed.index].push_back(
                            transformationRtMatrix(relativeTransformationSE3.inverse().matrix(), frameToToBeTransformed,
                                                   frameFromDestination));

                } else {

                    std::cout << "                             NOT ___success____ " << frameFromDestination.index
                              << " -> "
                              << frameToToBeTransformed.index << " \tmatches " << match.matchNumbers.size()
                              << std::endl;
                }
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
                                                            double maxProjectionErrorPixels) {
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
        auto inlierMatchesCorrespondingKeypointsLoRansac = findInlierPointCorrespondences(vertexFromDestOrigin,
                                                                                          vertexInListToBeTransformedCanBeComputed,
                                                                                          neighbourhoodRadius,
                                                                                          cR_t_umeyama,
                                                                                          false);

        bool successRefine = true;
        refineRelativePose(verticesOfCorrespondence[vertexToBeTransformed],
                           verticesOfCorrespondence[vertexFromDestOrigin], cR_t_umeyama, successRefine);


        auto inlierMatchesCorrespondingKeypointsAfterRefinement = findInlierPointCorrespondences(vertexFromDestOrigin,
                                                                                                 vertexInListToBeTransformedCanBeComputed,
                                                                                                 neighbourhoodRadius,
                                                                                                 cR_t_umeyama,
                                                                                                 true);

        Eigen::Matrix4d inverseICP = cR_t_umeyama.inverse();
        auto inlierMatchesCorrespondingKeypointsAfterRefinementInverse = findInlierPointCorrespondences(
                vertexFromDestOrigin,
                vertexInListToBeTransformedCanBeComputed,
                neighbourhoodRadius,
                inverseICP,
                true);

        ++totalMeausedRelativePoses;
        int ransacInliers = inlierMatchesCorrespondingKeypointsLoRansac.size();
        int ICPinliers = inlierMatchesCorrespondingKeypointsAfterRefinement.size();
        int ICPinliersInverse = inlierMatchesCorrespondingKeypointsAfterRefinementInverse.size();
        std::cout << "              " << "ransac got " << ransacInliers << "/" << toBeTransformedPoints.cols()
                  << " vs " << ICPinliers << " vs [inverse] " << ICPinliersInverse << std::endl;


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
            // ICP did refine the relative pose -- return ICP inliers
            std::cout << "REFINED________________________________________________" << std::endl;
            ++refinedPoses;
            std::swap(inlierMatchesCorrespondingKeypointsAfterRefinement, inlierMatchesCorrespondingKeypointsLoRansac);
        }

        for (const auto &matchPair: inlierMatchesCorrespondingKeypointsLoRansac) {
            inlierCorrespondencesPoints.push_back(matchPair);
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
                keysDescriptorsAll = siftModule.getKeypointsDescriptorsAllImages(readRgbData(pathToImageDirectoryRGB), {0});

        verticesOfCorrespondence.reserve(keysDescriptorsAll.size());

        for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {

            keyPointsDepthDescriptor keyPointsDepthDescriptor = filterKeypointsByKnownDepth(keysDescriptorsAll[currentImage], imagesD[currentImage]);
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

        matches = siftModule.findCorrespondences(verticesOfCorrespondence);
        decreaseDensity();
        findTransformationRtMatrices();
        computePointClasses();
        std::string poseFile = relativePose;
        printRelativePosesFile(poseFile);

        return 0;
    }

    CorrespondenceGraph::CorrespondenceGraph(const std::string &newPathToImageDirectoryRGB,
                                             const std::string &newPathToImageDirectoryD,
                                             float fx, float cx, float fy, float cy) :
            cameraRgbd({fx, cx, fy, cy}),
            pathToImageDirectoryRGB(newPathToImageDirectoryRGB),
            pathToImageDirectoryD(newPathToImageDirectoryD) {

        std::cout << "construct Graph" << std::endl;
        imagesRgb = readRgbData(pathToImageDirectoryRGB);
        imagesD = readRgbData(pathToImageDirectoryD);
        std::cout << "data have been read" << std::endl;

        std::sort(imagesRgb.begin(), imagesRgb.end());
        std::sort(imagesD.begin(), imagesD.end());


        PRINT_PROGRESS("images rgb vs d: " << imagesRgb.size() << " vs " << imagesD.size());
        assert(imagesRgb.size() == imagesD.size());

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

    std::vector<int> CorrespondenceGraph::bfs(int currentVertex) {
        std::vector<bool> visited(verticesOfCorrespondence.size(), false);
        std::vector<int> preds(verticesOfCorrespondence.size(), -1);
        std::queue<int> queueVertices;
        queueVertices.push(currentVertex);
        assert(verticesOfCorrespondence.size() == transformationRtMatrices.size());
        while (!queueVertices.empty()) {
            int vertex = queueVertices.front();
            PRINT_PROGRESS(" entered vertex " << vertex);
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
                    const Eigen::Matrix4d &predAbsoluteRt = verticesOfCorrespondence[vertex].absoluteRotationTranslation;
                    Eigen::Matrix3d predR = predAbsoluteRt.block(0, 0, 3, 3);
                    Eigen::Vector3d predT = predAbsoluteRt.block(0, 3, 3, 1);

                    const Eigen::Matrix4d &relativeRt = transformationRtMatrices[vertex][i].innerTranformationRtMatrix;
                    Eigen::Vector3d relT = relativeRt.block(0, 3, 3, 1);

                    Eigen::Matrix4d &newAbsoluteRt = verticesOfCorrespondence[to].absoluteRotationTranslation;
                    Eigen::Vector3d newAbsoluteT = predR * relT + predT;

                    newAbsoluteRt.block(0, 3, 3, 1) = newAbsoluteT;

                }
            }
        }
        return preds;
    }

    // each inner vector represents matches (usually 2) between keypoint: pose number and local point index & key point info

    void CorrespondenceGraph::computePointClasses() {
        computePointClasses(inlierCorrespondencesPoints);
    }

    void CorrespondenceGraph::computePointClasses(
            const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &matchesBetweenPoints) {


        for (const std::vector<std::pair<std::pair<int, int>, KeyPointInfo>> &vectorOfMatches: matchesBetweenPoints) {

            std::vector<std::pair<int, int>> poseAndLocalIndices;
            for (const std::pair<std::pair<int, int>, KeyPointInfo> &fullPointInfo: vectorOfMatches) {
                poseAndLocalIndices.push_back(fullPointInfo.first);
            }
            pointMatcher.insertPointsWithNewClasses(poseAndLocalIndices);
        }


        // unordered map's Key is local index
        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPoseNumAndLocalInd(
                pointMatcher.getNumberOfPoses());


        for (const std::vector<std::pair<std::pair<int, int>, KeyPointInfo>> &vectorOfMatches: matchesBetweenPoints) {

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
//        std::vector<Eigen::Quaterniond> absolutePosesQuat = optimizeRotationsRobust();
//
//        assert(absolutePoses.size() == absolutePosesQuat.size());
//        for (int i = 0; i < absolutePoses.size(); ++i) {
//            absolutePoses[i].block<3,3>(0,0) = absolutePosesQuat[i].toRotationMatrix();
//        }

        for (int indexFrom = 0; indexFrom < verticesOfCorrespondence.size(); ++indexFrom) {
            for (const auto &knownRelativePose: transformationRtMatrices[indexFrom]) {
                assert(indexFrom == knownRelativePose.getIndexFrom());

//                assert(knownRelativePose.getIndexFrom() < knownRelativePose.getIndexTo());
                // changed order here of indexFrom indexTo


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
}
