#pragma once
#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <pcl/registration/incremental_registration.h>

#include <queue>

#include "util.h"
#include "VertexCG.h"
#include "transformationRt.h"
#include "cameraRGBD.h"
#include "siftModule.h"
#include "images.h"
#include "rotationAveraging.h"
#include "quaternions.h"

#include <opencv2/opencv.hpp>

typedef struct Match {
    int frameNumber;
    std::vector<std::pair<int, int>> matchNumbers;
//    int keypointNumber;
//    int correspondingKeypointNumber;

    Match(int newFrameNumber, const std::vector<std::pair<int, int>> &newMatchNumbers) :
            frameNumber(newFrameNumber),
            matchNumbers(newMatchNumbers) {};
} Match;


typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

typedef struct CorrespondenceGraph {
    CameraRGBD cameraRgbd;
    SiftModule siftModule;
    std::vector<int> originVerticesNumbers;
    std::vector<VertexCG> verticesOfCorrespondence;
    int maxVertexDegree = 20;
    int numIterations = 50;
    std::vector<std::vector<Match>> matches;
    std::vector<std::vector<transformationRtMatrix>> tranformationRtMatrices;
    double neighbourhoodRadius = 0.05;
    int minNumberOfInliersAfterRobust = 10;
    const std::string redCode = "\033[0;31m";
    const std::string resetCode = "\033[0m";
    const std::string relativePose = "relativeRotations.txt";
    const std::string absolutePose = "absoluteRotations.txt";

    CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD, float fx,
                        float cx, float fy, float cy);

    int findCorrespondences();

    int findCorrespondencesEveryDepth();

    int findTransformationRtMatrices();

    int findRotationsTranslations();

    int findRotationTranslation(int vertexFrom, int vertexInList);

    void decreaseDensity();

    MatrixX getTransformationRtMatrixTwoImages(int vertexFrom, int vertexInList, MatrixX &outR, MatrixX &outT, bool &success,
                                               double inlierCoeff = 0.75);

    cv::Mat getEssentialMatrixTwoImagesMatched(int vertexFrom, int vertexTo);

    void showKeypointsOnDephtImage(int vertexFrom);

    MatrixX
    getTransformationMatrixUmeyamaLoRANSAC(const MatrixX &toBeTransormedPoints, const MatrixX &destinationPoints, const int numIterations,
                                           const int numOfElements, double inlierCoeff);

    void printConnections(std::ostream &os, int space = 10);

    std::vector<int> bfs(int currentVertex);
} CorrespondenceGraph;

#endif
