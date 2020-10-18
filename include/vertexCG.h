#pragma once

#ifndef TEST_SIFTGPU_VERTEXCG_H
#define TEST_SIFTGPU_VERTEXCG_H

#include <vector>
#include <iostream>
#include <SiftGPU.h>

typedef struct {

    int index;
    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<double> descriptors;
    std::vector<double> depths;
    std::string pathToRGBimage;
    std::string pathToDimage;
} vertexCG;

#endif //TEST_SIFTGPU_VERTEXCG_H
