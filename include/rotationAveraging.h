#pragma once
#ifndef TEST_SIFTGPU_ROTATIONAVERAGING_H
#define TEST_SIFTGPU_ROTATIONAVERAGING_H

#include <string>
struct rotationAverager {
    static int shanonAveraging(const std::string& pathToRelativeRotations, const std::string& pathOut);
};
#endif
