#pragma once
#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H

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

typedef struct CorrespondenceGraph {
    CameraRGBD cameraRgbd;
    SiftModule siftModule;
    std::vector<vertexCG> verticesOfCorrespondence;
    int maxVertexDegree = 5;
    std::vector<std::vector<Match>> matches;
    std::vector<std::vector<essentialMatrix>> essentialMatrices;
    CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD, float fx, float cx, float fy, float cy);

    int findCorrespondences();
    int findEssentialMatrices();
    int findRotationsTranslations();
    int findRotationTranslation(int vertexFrom, int vertexInList);
    void decreaseDensity();
    cv::Mat getEssentialMatrixTwoImages(int vertexFrom, int vertexInList, cv::Mat& outR, cv::Mat& outT);
    cv::Mat getEssentialMatrixTwoImagesMatched(int vertexFrom, int vertexTo);
} CorrespondenceGraph;

#endif //TEST_SIFTGPU_CG_H
