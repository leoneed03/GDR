#pragma once
#ifndef TEST_SIFTGPU_FEATURES_H
#define TEST_SIFTGPU_FEATURES_H

#include <vector>
#include <SiftGPU.h>
#include <string>
#include <iostream>

#include "files.h"

using imageDescriptor = std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>;

std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> getKeypointsDescriptorsOneImage(
        SiftGPU &sift,
        const std::string &pathToTheImage);

std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> getKeypointsDescriptorsAllImages(
        SiftGPU &sift,
        const std::string &pathToTheDirectory);

std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<SiftGPU::SiftKeypoint>>
getMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                    const imageDescriptor &keysDescriptors2,
                    SiftMatchGPU *matcher);

std::vector<std::pair<int,int>>
getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                             const imageDescriptor &keysDescriptors2,
                             SiftMatchGPU *matcher);

#endif //TEST_SIFTGPU_FEATURES_H
