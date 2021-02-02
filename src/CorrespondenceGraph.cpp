//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CorrespondenceGraph.h"
#include "printer.h"
#include "ICP.h"
#include "pointCloud.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

namespace gdr {


    int
    CorrespondenceGraph::refineRelativePose(const VertexCG &vertexToBeTransformed, const VertexCG &vertexDestination,
                                            Eigen::Matrix4d &initEstimationRelPos, bool &success) {
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
                                                        Eigen::Matrix4d &transformation) {
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

    int CorrespondenceGraph::findCorrespondences() {

        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            for (int j = i + 1; j < verticesOfCorrespondence.size(); ++j) {

                PRINT_PROGRESS("currently " << i << ' ' << j);
                std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                        std::make_pair(verticesOfCorrespondence[i].keypoints, verticesOfCorrespondence[i].descriptors),
                        std::make_pair(verticesOfCorrespondence[j].keypoints, verticesOfCorrespondence[j].descriptors),
                        siftModule.matcher.get());
                PRINT_PROGRESS("total matches " << matchingNumbers.size() << std::endl);
                matches[i].push_back({j, matchingNumbers});
            }
        }

        return 0;
    }

    int CorrespondenceGraph::findTransformationRtMatrices() {

        for (int i = 0; i < matches.size(); ++i) {
            for (int j = 0; j < matches[i].size(); ++j) {

                const auto &match = matches[i][j];
                const auto &frameFrom = verticesOfCorrespondence[i];
                const auto &frameTo = verticesOfCorrespondence[match.frameNumber];
                PRINT_PROGRESS("check this " << frameFrom.index << " -> " << frameTo.index);
                assert(frameTo.index > frameFrom.index);
                bool success = true;
                bool successICP = true;
                auto cameraMotion = getTransformationRtMatrixTwoImages(i, j, success);


                PRINT_PROGRESS(
                        "out of Transformation calculation" << std::endl
                                                            << frameFrom.index << " -> " << frameTo.index);

                if (success) {
                    int spaceIO = 18;
                    std::cout << "success " << frameFrom.index << " -> " << frameTo.index << std::endl;


                    /////use ICPCUDA to refine estimation
                    refineRelativePose(frameFrom, frameTo, cameraMotion, successICP);

                    Eigen::Matrix3d m3d = cameraMotion.block(0, 0, 3, 3);
                    Eigen::Quaterniond qRelatived(m3d);

                    PRINT_PROGRESS(std::setw(2 * spaceIO) << qRelatived.x() << std::setw(2 * spaceIO) << qRelatived.y()
                                                          << std::setw(2 * spaceIO) << qRelatived.z()
                                                          << std::setw(2 * spaceIO) << qRelatived.w());

                    transformationRtMatrices[i].push_back(transformationRtMatrix(cameraMotion, frameFrom, frameTo));
                    transformationRtMatrices[frameTo.index].push_back(
                            transformationRtMatrix(cameraMotion.inverse(), frameTo, frameFrom));
                } else {

                    std::cout << "                             NOT ___success____ " << frameFrom.index << " -> "
                              << frameTo.index << " \tmatches " << match.matchNumbers.size() << std::endl;
                    PRINT_PROGRESS("transformation matrix not found");
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
    CorrespondenceGraph::getTransformationRtMatrixTwoImages(int vertexFrom,
                                                            int vertexInList,
                                                            bool &success,
                                                            double inlierCoeff) {
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
        {
            const auto &match = matches[vertexFrom][vertexInList];
            int minSize = match.matchNumbers.size();
            if (minSize < minNumberOfInliersAfterRobust / inlierCoeff) {
                success = false;
                return cR_t_umeyama;
            }


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

                double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
                int localIndexToBeTransformed = match.matchNumbers[i].second;
                int vertexToBeTransformed = match.frameNumber;
                const auto &siftKeyPointToBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].keypoints[localIndexToBeTransformed];
                x_toBeTransformed = siftKeyPointToBeTransformed.x;
                y_toBeTransformed = siftKeyPointToBeTransformed.y;
                z_toBeTransformed = verticesOfCorrespondence[vertexToBeTransformed].depths[localIndexToBeTransformed];
                toBeTransformedPointsVector.push_back({x_toBeTransformed, y_toBeTransformed, z_toBeTransformed, 1});

                std::vector<std::pair<int, int>> points = {{vertexFrom,            localIndexOrigin},
                                                           {vertexToBeTransformed, localIndexToBeTransformed}};

            }
            assert(toBeTransformedPointsVector.size() == minSize);
            assert(originPointsVector.size() == minSize);


            Eigen::Matrix4Xd toBeTransformedPoints = getPointCloudBeforeProjection(toBeTransformedPointsVector,
                                                                                   verticesOfCorrespondence[vertexFrom].cameraRgbd);
            Eigen::Matrix4Xd originPoints = getPointCloudBeforeProjection(originPointsVector,
                                                                          verticesOfCorrespondence[match.frameNumber].cameraRgbd);
            assert(toBeTransformedPoints.cols() == minSize);
            assert(originPoints.cols() == minSize);

            cR_t_umeyama = getTransformationMatrixUmeyamaLoRANSAC(toBeTransformedPoints,
                                                                  originPoints,
                                                                  numIterations,
                                                                  minSize,
                                                                  inlierCoeff,
                                                                  success,
                                                                  neighbourhoodRadius);
            if (!success) {
                PRINT_PROGRESS("no stable transformation matrix estimation was found with umeyama");
                cR_t_umeyama.setIdentity();
                return cR_t_umeyama;
            }


            PRINT_PROGRESS("RANSAC umeyama " << std::endl
                                             << cR_t_umeyama << std::endl
                                             << "______________________________________________________\n"
                                             << "______________________________________________________\n");
        }

        PRINT_PROGRESS("return success -- transformation matrix found");




        // look for inliers after umeyama
        const auto &inlierMatchesCorrespondingKeypoints = findInlierPointCorrespondences(vertexFrom,
                                                                                         vertexInList,
                                                                                         neighbourhoodRadius,
                                                                                         cR_t_umeyama);
        for (const auto& matchPair: inlierMatchesCorrespondingKeypoints) {
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
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)(gtsam format)
                    file << "EDGE_SE3:QUAT " << indexTo << ' ' << indexFrom << ' ';
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
        std::vector<Eigen::Matrix3d> absoluteRotations;
        for (const auto &absoluteRotationQuat: absoluteRotationsQuats) {
            absoluteRotations.push_back(absoluteRotationQuat.toRotationMatrix());
        }

        PRINT_PROGRESS("get Rotations from quaternions successfull");
        for (int i = 0; i < verticesOfCorrespondence.size(); ++i) {
            verticesOfCorrespondence[i].setRotation(absoluteRotations[i]);
        }

        PRINT_PROGRESS("setting Rotations in vertices successfull");
        return absoluteRotationsQuats;
    }

    int CorrespondenceGraph::computeRelativePoses() {

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
                keysDescriptorsAll = getKeypointsDescriptorsAllImages(siftModule.sift, pathToImageDirectoryRGB);

        verticesOfCorrespondence.reserve(keysDescriptorsAll.size());

        for (int currentImage = 0; currentImage < keysDescriptorsAll.size(); ++currentImage) {

            auto keypointAndDescriptor = keysDescriptorsAll[currentImage];
            std::vector<SiftGPU::SiftKeypoint> &keypoints = keypointAndDescriptor.first;
            std::vector<float> &descriptors = keypointAndDescriptor.second;
            std::vector<SiftGPU::SiftKeypoint> keypointsKnownDepth;
            std::vector<float> descriptorsKnownDepth;
            std::vector<double> depths;

            cv::Mat depthImage = cv::imread(imagesD[currentImage], cv::IMREAD_ANYDEPTH);
            PRINT_PROGRESS(depthImage.cols << ' ' << depthImage.rows);

            for (int i = 0; i < keypoints.size(); ++i) {
                int posInDescriptorVector = 128 * i;
                int currentKeypointDepth = depthImage.at<ushort>(keypoints[i].y, keypoints[i].x);

                if (currentKeypointDepth > 0) {
                    assert(currentKeypointDepth < 66000);
                    depths.push_back(currentKeypointDepth / 5000.0);
                    keypointsKnownDepth.push_back(keypoints[i]);
                    std::vector<float> currentDescriptors;
                    for (int descriptorCounter = 0; descriptorCounter < 128; ++descriptorCounter) {
                        descriptorsKnownDepth.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                        currentDescriptors.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                    }
                }
            }

            VertexCG currentVertex(currentImage, cameraRgbd, keypointsKnownDepth, descriptorsKnownDepth, depths,
                                   imagesRgb[currentImage],
                                   imagesD[currentImage]);
            verticesOfCorrespondence.push_back(currentVertex);
            assert(keypointsKnownDepth.size() == depths.size());
            assert(verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].depths.size() ==
                   verticesOfCorrespondence[verticesOfCorrespondence.size() - 1].keypoints.size());
        }

        PRINT_PROGRESS("vertices written");
        matches = std::vector<std::vector<Match>>(verticesOfCorrespondence.size());

        PRINT_PROGRESS("trying to find correspondences");

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
        findCorrespondences();
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

        imagesRgb = readRgbData(pathToImageDirectoryRGB);
        imagesD = readRgbData(pathToImageDirectoryD);

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
            keyPointInfo.push_back(keyPointInfoByPoseNumAndLocalInd[poseNumberAndLocalIndex.first][poseNumberAndLocalIndex.second]);
            cloudProjector.addPoint(pointClassNumber, keyPointInfo);
        }


    }

    const CloudProjector &CorrespondenceGraph::getCloudProjector() const {
        return cloudProjector;
    }
}
