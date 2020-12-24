//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_FEATURES_H
#define TEST_SIFTGPU_FEATURES_H

#include <vector>
#include <SiftGPU.h>
#include <string>
#include <iostream>

#include "fileProc.h"

namespace gdr {

    using imageDescriptor = std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>;

    std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> getKeypointsDescriptorsOneImage(
            SiftGPU &sift,
            const std::string &pathToTheImage);

    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
    getKeypointsDescriptorsAllImages(
            SiftGPU
            &sift,
            const std::string &pathToTheDirectory
    );

    std::vector<std::pair<int, int>>
    getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                                 const imageDescriptor &keysDescriptors2,
                                 SiftMatchGPU *matcher);
}

#endif
