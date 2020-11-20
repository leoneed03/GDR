#pragma once
#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD

#include "vertexCG.h"
#include "essentialMatrix.h"
#include "cameraRGBD.h"
#include "siftModule.h"
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
    std::vector<vertexCG> verticesOfCorrespondence;
    int maxVertexDegree = 5;
    std::vector<std::vector<Match>> matches;
    std::vector<std::vector<essentialMatrix>> essentialMatrices;

    CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD, float fx,
                        float cx, float fy, float cy);

    int findCorrespondences();

    int findCorrespondencesEveryDepth();

    int findEssentialMatrices();

    int findRotationsTranslations();

    int findRotationTranslation(int vertexFrom, int vertexInList);

    void decreaseDensity();

    cv::Mat getEssentialMatrixTwoImagesOpenCV(int vertexFrom, int vertexInList, cv::Mat &outR, cv::Mat &outT);

    MatrixX getEssentialMatrixTwoImages(int vertexFrom, int vertexInList, MatrixX &outR, MatrixX &outT);

    cv::Mat getEssentialMatrixTwoImagesMatched(int vertexFrom, int vertexTo);

    void showKeypointsOnDephtImage(int vertexFrom);
    MatrixX getTransformationMatrixUmeyamaLoRANSAC(const MatrixX& points1, const MatrixX& points2,  const int numIterations, const int numOfElements, double inlierCoeff);
} CorrespondenceGraph;

#endif //TEST_SIFTGPU_CG_H
