//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"
#include "printer.h"
#include "ICP.h"
#include "pointCloud.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>


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

                    tranformationRtMatrices[i].push_back(transformationRtMatrix(cameraMotion, frameFrom, frameTo));
                    tranformationRtMatrices[frameTo.index].push_back(
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
    CorrespondenceGraph::getTransformationRtMatrixTwoImages(int vertexFrom, int vertexInList,
                                                            bool &success, double inlierCoeff) {
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
                {
                    double x_origin, y_origin, z_origin;
                    x_origin = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].x;
                    y_origin = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].y;
                    z_origin = verticesOfCorrespondence[vertexFrom].depths[match.matchNumbers[i].first];
                    originPointsVector.push_back({x_origin, y_origin, z_origin, 1});
                }
                {
                    double x_toBeTransformed, y_toBeTransformed, z_toBeTransformed;
                    x_toBeTransformed = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].x;
                    y_toBeTransformed = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].y;
                    z_toBeTransformed = verticesOfCorrespondence[match.frameNumber].depths[match.matchNumbers[i].second];
                    toBeTransformedPointsVector.push_back({x_toBeTransformed, y_toBeTransformed, z_toBeTransformed, 1});
                }
            }
            assert(toBeTransformedPointsVector.size() == minSize);
            assert(originPointsVector.size() == minSize);


            Eigen::Matrix4Xd toBeTransformedPoints = getPointCloudBeforeProjection(toBeTransformedPointsVector,
                                                                                   verticesOfCorrespondence[vertexFrom].cameraRgbd);
            Eigen::Matrix4Xd originPoints = getPointCloudBeforeProjection(originPointsVector,
                                                                          verticesOfCorrespondence[match.frameNumber].cameraRgbd);
            assert(toBeTransformedPoints.cols() == minSize);
            assert(originPoints.cols() == minSize);




            //old version start

            /*
            Eigen::Matrix4Xd toBeTransformedPoints(4, minSize);
            Eigen::Matrix4Xd originPoints(4, minSize);

            double mz = 1000;
            double Mz = -1000;
            for (int i = 0; i < minSize; ++i) {
                {
                    double x1, y1, z1;
                    x1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].x;
                    y1 = verticesOfCorrespondence[vertexFrom].keypoints[match.matchNumbers[i].first].y;
                    z1 = verticesOfCorrespondence[vertexFrom].depths[match.matchNumbers[i].first];

                    double mirrorParameterH = verticesOfCorrespondence[vertexFrom].heightMirrorParameter;
                    assert(y1 < mirrorParameterH && y1 > 0);
                    y1 = mirrorParameterH - y1;

                    double mirrorParameterW = verticesOfCorrespondence[vertexFrom].widthMirrorParameter;
                    assert(x1 < mirrorParameterW && x1 > 0);
                    x1 = mirrorParameterW - x1;

                    x1 = 1.0 * (x1 - cameraRgbd.cx) * z1 / cameraRgbd.fx;
                    y1 = 1.0 * (y1 - cameraRgbd.cy) * z1 / cameraRgbd.fy;

                    if (z1 < mz) {
                        mz = z1;
                    }
                    if (z1 > Mz) {
                        Mz = z1;
                    }


                    assert(z1 > 3 * std::numeric_limits<double>::epsilon());
                    assert(z1 < 14);
                    originPoints.col(i) << x1, y1, z1, 1;
                }

                {
                    double x2, y2, z2;
                    x2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].x;
                    y2 = verticesOfCorrespondence[match.frameNumber].keypoints[match.matchNumbers[i].second].y;
                    z2 = verticesOfCorrespondence[match.frameNumber].depths[match.matchNumbers[i].second];

                    double mirrorParameterH = verticesOfCorrespondence[vertexFrom].heightMirrorParameter;
                    assert(y2 < mirrorParameterH && y2 >= 0);
                    y2 = mirrorParameterH - y2;

                    double mirrorParameterW = verticesOfCorrespondence[vertexFrom].widthMirrorParameter;
                    assert(x2 < mirrorParameterW && x2 > 0);
                    x2 = mirrorParameterW - x2;

                    x2 = 1.0 * (x2 - cameraRgbd.cx) * z2 / cameraRgbd.fx;
                    y2 = 1.0 * (y2 - cameraRgbd.cy) * z2 / cameraRgbd.fy;

                    assert(z2 > 3 * std::numeric_limits<double>::epsilon());
                    assert(z2 < 14);
                    toBeTransformedPoints.col(i) << x2, y2, z2, 1;
                }
            }

            assert(mz > 0);
            assert(Mz > 0);


            //old version end
             */

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
        return cR_t_umeyama;
    }

    int CorrespondenceGraph::printRelativePosesFile(const std::string &pathOutRelativePoseFile) {

        std::ofstream file(pathOutRelativePoseFile);

        if (file.is_open()) {
            int numPoses = tranformationRtMatrices.size();
            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }
            for (int i = 0; i < tranformationRtMatrices.size(); ++i) {
                for (int j = 0; j < tranformationRtMatrices[i].size(); ++j) {
                    if (i >= tranformationRtMatrices[i][j].vertexTo.index) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = tranformationRtMatrices[i][j].vertexTo.index;
                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)(gtsam format)
                    file << "EDGE_SE3:QUAT " << indexTo << ' ' << indexFrom << ' ';
                    auto translationVector = tranformationRtMatrices[i][j].t;
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &R = tranformationRtMatrices[i][j].R;

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
//
//        PRINT_PROGRESS("Shonan averaging successfull");
//        std::vector<std::vector<double>> quaternions = parseAbsoluteRotationsFile(absolutePose);

        PRINT_PROGRESS("read quaternions successfull");
        std::vector<Eigen::Matrix3d> absoluteRotations;
        for (const auto& absoluteRotationQuat: absoluteRotationsQuats) {
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
        findCorrespondences();
        decreaseDensity();
        findTransformationRtMatrices();
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

        tranformationRtMatrices = std::vector<std::vector<transformationRtMatrix>>(imagesD.size());

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
        for (int i = 0; i < tranformationRtMatrices.size(); ++i) {
            os << std::setw(space / 5) << i << ":";
            counter += tranformationRtMatrices[i].size();
            counterSquared += tranformationRtMatrices[i].size() * tranformationRtMatrices[i].size();
            for (int j = 0; j < tranformationRtMatrices[i].size(); ++j) {
                const transformationRtMatrix &e = tranformationRtMatrices[i][j];
                assert(i == e.vertexFrom.index);
                os << std::setw(space / 2) << e.vertexTo.index << ",";
            }
            os << std::endl;
        }
        os << "average number of edges " << counter / tranformationRtMatrices.size() << std::endl;

        os << "sq D " << sqrt(counterSquared * 1.0 / tranformationRtMatrices.size() -
                              pow(counter * 1.0 / tranformationRtMatrices.size(), 2)) << std::endl;

    }

    std::vector<int> CorrespondenceGraph::bfs(int currentVertex) {
        std::vector<bool> visited(verticesOfCorrespondence.size(), false);
        std::vector<int> preds(verticesOfCorrespondence.size(), -1);
        std::queue<int> queueVertices;
        queueVertices.push(currentVertex);
        assert(verticesOfCorrespondence.size() == tranformationRtMatrices.size());
        while (!queueVertices.empty()) {
            int vertex = queueVertices.front();
            PRINT_PROGRESS(" entered vertex " << vertex);
            queueVertices.pop();
            assert(vertex < visited.size() && vertex >= 0);
            visited[vertex] = true;

            for (int i = 0; i < tranformationRtMatrices[vertex].size(); ++i) {
                int to = tranformationRtMatrices[vertex][i].vertexTo.index;
                if (!visited[to]) {
                    queueVertices.push(to);
                    visited[to] = true;
                    assert(preds[to] == -1);
                    preds[to] = vertex;

                    ///// get absolute Rotation (R) and Translation (t) with given predecessor Vertex and Relative R & t
                    const Eigen::Matrix4d &predAbsoluteRt = verticesOfCorrespondence[vertex].absoluteRotationTranslation;
                    Eigen::Matrix3d predR = predAbsoluteRt.block(0, 0, 3, 3);
                    Eigen::Vector3d predT = predAbsoluteRt.block(0, 3, 3, 1);

                    const Eigen::Matrix4d &relativeRt = tranformationRtMatrices[vertex][i].innerTranformationRtMatrix;
                    Eigen::Vector3d relT = relativeRt.block(0, 3, 3, 1);

                    Eigen::Matrix4d &newAbsoluteRt = verticesOfCorrespondence[to].absoluteRotationTranslation;
                    Eigen::Vector3d newAbsoluteT = predR * relT + predT;

                    newAbsoluteRt.block(0, 3, 3, 1) = newAbsoluteT;

                }
            }
        }
        return preds;
    }
}
