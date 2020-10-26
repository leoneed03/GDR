#include "../include/siftModule.h"

SiftModule::SiftModule() {
    matcher = std::unique_ptr<SiftMatchGPU>(new SiftMatchGPU(maxSift));
}