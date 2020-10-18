#pragma once
#ifndef TEST_SIFTGPU_CG_H
#define TEST_SIFTGPU_CG_H
#include "vertexCG.h"

typedef struct CorrespondenceGraph {

    std::vector<vertexCG*> verticesOfCorrespondence;
    int maxVertexDegree = 80;
    std::vector<std::vector<std::vector<std::pair<int,int>>>> keypointsMatches;
    CorrespondenceGraph(const std::string& pathToImageDirectoryRGB, const std::string& pathToImageDirectoryD);
} CorrespondenceGraph;

#endif //TEST_SIFTGPU_CG_H
