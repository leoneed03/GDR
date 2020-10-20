#pragma once

#ifndef TEST_SIFTGPU_VERTEXCG_H
#define TEST_SIFTGPU_VERTEXCG_H

#include <vector>
#include <iostream>
#include <SiftGPU.h>
#include "features.h"

typedef struct vertexCG {

    int index;
    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<float> descriptors;
    std::vector<int> depths; //TODO
    std::string pathToRGBimage;
    std::string pathToDimage;

    vertexCG(int newIndex,
             const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
             const std::vector<float> &newDescriptors,
             const std::vector<int> &newDepths,
             const std::string &newPathRGB,
             const std::string &newPathD);
} vertexCG;

#endif //TEST_SIFTGPU_VERTEXCG_H
