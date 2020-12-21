//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>


#include <queue>

#include "util.h"
#include "VertexCG.h"
#include "transformationRt.h"
#include "cameraRGBD.h"
#include "siftModule.h"
#include "images.h"
#include "rotationAveraging.h"
#include "quaternions.h"
#include "errors.h"

#include <opencv2/opencv.hpp>

struct Match {
    int frameNumber;
    std::vector<std::pair<int, int>> matchNumbers;

    Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
            frameNumber(newFrameNumber),
            matchNumbers(newMatchNumbers) {};
};

struct KeypointsAndDescriptors {
    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> pairsOfKeypointsAndDescriptors;
    KeypointsAndDescriptors(const std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>& newPairskeypointsAndDescriptors);
};

typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

struct CorrespondenceGraph {
    CameraRGBD cameraRgbd;
    SiftModule siftModule;
    std::vector<int> originVerticesNumbers;
    std::vector<VertexCG> verticesOfCorrespondence;
    int maxVertexDegree = 80;
    int numIterations = 50;
    std::vector<std::vector<Match>> matches;
    std::vector<std::vector<transformationRtMatrix>> tranformationRtMatrices;
    double neighbourhoodRadius = 0.05;
    int minNumberOfInliersAfterRobust = 10;
    const std::string redCode = "\033[0;31m";
    const std::string resetCode = "\033[0m";
    const std::string relativePose = "relativeRotations.txt";
    const std::string absolutePose = "absoluteRotations.txt";
    std::vector<std::string> imagesRgb;
    std::vector<std::string> imagesD;
    std::string pathToImageDirectoryRGB;
    std::string pathToImageDirectoryD;


    CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD, float fx,
                        float cx, float fy, float cy);

    int findCorrespondences();

    int findTransformationRtMatrices();

    void decreaseDensity();

    Eigen::Matrix4d
    getTransformationRtMatrixTwoImages(int vertexFrom, int vertexInList, bool &success,
                                       double inlierCoeff = 0.6);

    void showKeypointsOnDephtImage(int vertexFrom);


    void printConnectionsRelative(std::ostream &os, int space = 10);
    int printAbsolutePoses(std::ostream &os, int space = 10);
    int performRotationAveraging();

    int printRelativePosesFile(const std::string& outPutFileRelativePoses);

    std::vector<int> bfs(int currentVertex);

    int computeRelativePoses();
};


Eigen::Matrix4d
getTransformationMatrixUmeyamaLoRANSAC(const MatrixX &toBeTransormedPoints, const MatrixX &destinationPoints,
                                       const int numIterationsRansac,
                                       const int numOfElements, double inlierCoeff);

#endif
