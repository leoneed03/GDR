#pragma once

#ifndef TEST_SIFTGPU_VERTEXCG_H
#define TEST_SIFTGPU_VERTEXCG_H

#include <vector>
#include <iostream>
#include <SiftGPU.h>
#include "features.h"

typedef struct keypointWithDepth {
    SiftGPU::SiftKeypoint keypoint;
    double depth;
    std::vector<float> descriptors;
    keypointWithDepth(SiftGPU::SiftKeypoint newKeypoint, double newDepth, const std::vector<float>& newDescriptors);
} keypointWithDepth;

typedef struct vertexCG {

    int index;
    std::vector<keypointWithDepth> keypointsWithDepths;
    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<float> descriptors;
    std::vector<double> depths;
    std::string pathToRGBimage;
    std::string pathToDimage;
    int heightMirrorParameter = 480;
    int widthMirrorParameter = 640;

    vertexCG(int newIndex,
             const std::vector<keypointWithDepth> &newKeypointsWithDepths,
             const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
             const std::vector<float> &newDescriptors,
             const std::vector<double> &newDepths,
             const std::string &newPathRGB,
             const std::string &newPathD);
} vertexCG;

#endif //TEST_SIFTGPU_VERTEXCG_H
