#ifndef SIFTGPU_SIFT_H
#define SIFTGPU_SIFT_H

#include <memory>
#include "SiftGPU.h"

struct SiftModule {
    std::unique_ptr<SiftMatchGPU> matcher;
    SiftGPU sift;
    int maxSift = 4096;
    SiftModule();
};

#endif
