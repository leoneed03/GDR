#pragma once
#ifndef TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H
#define TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H

#include <string>

typedef struct GTT {
    static int makeRotationsRelative(const std::string& pathToGroundTruth, const std::string& pathToRelativeGroundTruth);
} GTT;
#endif
