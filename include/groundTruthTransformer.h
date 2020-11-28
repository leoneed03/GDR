#pragma once
#ifndef TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H
#define TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H

#include <string>
#include <set>
#include "files.h"

typedef struct GTT {
    static int makeRotationsRelative(const std::string& pathToGroundTruth, const std::string& pathToRelativeGroundTruth);
    static int makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB, const std::string &pathToD, const std::string &pathOutDirectory, const std::set<int> indices);
    } GTT;
#endif
