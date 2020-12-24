//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#define DEBUG_PRINT_SIFT_GPU 0

#include "siftModule.h"
#include <iostream>

void gdr::SiftModule::siftParseParams(std::vector<char *> &siftGpuArgs) {
    sift.ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
}

gdr::SiftModule::SiftModule() {

    std::vector<char *> siftGpuArgs = {"-cuda", "-fo", "-1", "-v", "0"};
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    if (DEBUG_PRINT_SIFT_GPU) {
        std::cout << "Parse params for sift default" << std::endl;
    }
    siftParseParams(siftGpuArgs);
    matcher->VerifyContextGL();
}
