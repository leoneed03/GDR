//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "siftModule.h"
#include "printer.h"

#include <iostream>

void gdr::SiftModule::siftParseParams(std::vector<char *> &siftGpuArgs) {
    sift.ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
}

gdr::SiftModule::SiftModule() {

    std::vector<char *> siftGpuArgs = {"-cuda", "-fo", "-1", "-v", "0"};
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
    PRINT_PROGRESS("Parse params for sift default");
    siftParseParams(siftGpuArgs);
    matcher->VerifyContextGL();
}
