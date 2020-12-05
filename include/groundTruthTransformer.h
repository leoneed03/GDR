#pragma once
#ifndef TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H
#define TEST_SIFTGPU_GROUNDTRUTHTRANSFORMER_H

#include <string>
#include <set>
#include <unordered_map>
#include <map>
#include "files.h"

typedef struct GTT {
    static std::vector<double> createTimestamps(const std::vector<std::string>& rgb, const std::string &pathTimeRGB, const std::string &pathToGroundTruth, const std::set<int>& indices);
    static std::vector<std::vector<double>> getGroundTruth(const std::string& pathToGroundTruth, const std::vector<double>& timeStamps);
    static int makeRotationsRelative(const std::string& pathToGroundTruth, const std::string& pathToRelativeGroundTruth);
    static int writeGroundTruth(const std::string& pathOut, const std::vector<std::vector<double>>& timeCoordinates);
    static std::pair<std::vector<std::string>, std::vector<std::string>> makeRotationsRelativeAndExtractImages(const std::string &pathToGroundTruth, const std::string &pathToRGB, const std::string &pathToD, const std::string &pathOutDirectory, const std::string& timeInfo, const std::set<int> indices);
    static void writeInfo(const std::vector<std::string>& rgb, const std::string &pathTimeRGB, const std::string &pathToGroundTruth, const std::string& pathOut, const std::set<int>& indices);
    static void prepareDataset(const std::string& pathToDataset, const std::string& pathOut, const std::set<int>& chosenIndices, const std::string NewName);
} GTT;
#endif
