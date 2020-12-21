//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/siftModule.h"
#include <iostream>

void SiftModule::siftParseParams(std::vector<char *> &siftGpuArgs) {
    sift.ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
}
SiftModule::SiftModule(std::vector<char *> &siftGpuArgs) {
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    std::cout << "Parse params for sift specified" << std::endl;
    siftParseParams(siftGpuArgs);
    matcher->VerifyContextGL();
}
SiftModule::SiftModule() {
    std::vector<char*> siftGpuArgs = {"-cuda", "-fo", "-1", "-v", "1"};
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    std::cout << "Parse params for sift default" << std::endl;
    siftParseParams(siftGpuArgs);
    matcher->VerifyContextGL();
}
