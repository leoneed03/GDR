#pragma once
#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H

#include "vertexCG.h"

typedef struct Match {
    int frameNumber;
    std::vector<std::pair<int,int>> matchNumbers;
//    int keypointNumber;
//    int correspondingKeypointNumber;

    Match(int newFrameNumber, const std::vector<std::pair<int,int>>& newMatchNumbers) :
            frameNumber(newFrameNumber),
            matchNumbers(newMatchNumbers)
            {};
} Match;

typedef struct CorrespondenceGraph {
    std::unique_ptr<SiftMatchGPU> matcher;
    SiftGPU sift;
    int maxSift = 4096;
    std::vector<vertexCG> verticesOfCorrespondence;
    std::vector<std::vector<int>> correspondences;
    int maxVertexDegree = 10;
//    std::vector<imageDescriptor> allKeysDescriptors;
    std::vector<std::vector<std::vector<std::pair<int, int>>>> keypointsMatches;
    std::vector<std::vector<Match>> matches;
    CorrespondenceGraph(const std::string &pathToImageDirectoryRGB, const std::string &pathToImageDirectoryD);

    int findCorrespondences();
    void decreaseDensity();
} CorrespondenceGraph;

#endif //TEST_SIFTGPU_CG_H
