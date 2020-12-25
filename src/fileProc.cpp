//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "fileProc.h"
#include "printer.h"
#include "errors.h"

#include <algorithm>
#include <fstream>

std::vector<std::string> gdr::readRgbData(std::string pathToRGB) {
    DIR *pDIR;
    struct dirent *entry;
    std::vector<std::string> RgbImages;
    PRINT_PROGRESS("start reading images");
    if ((pDIR = opendir(pathToRGB.data())) != nullptr) {
        while ((entry = readdir(pDIR)) != nullptr) {
            if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
                continue;
            }
            std::string absolutePathToRgbImage = pathToRGB + "/" + entry->d_name;
            RgbImages.emplace_back(absolutePathToRgbImage);
        }
        closedir(pDIR);
    } else {
        std::cerr << "Unable to open" << std::endl;
        exit(ERROR_OPENING_FILE_READ);
    }
    std::sort(RgbImages.begin(), RgbImages.end());
    return RgbImages;
}

std::vector<std::vector<double>> gdr::parseAbsoluteRotationsFile(const std::string &pathToRotationsFile) {
    std::ifstream in(pathToRotationsFile);
    std::vector<std::vector<double>> quaternions;
    int numOfEmptyLines = 0;
    int numbersInLine = 8;

    if (in) {
        for (int i = 0; i < numOfEmptyLines; ++i) {
            std::string s;
            std::getline(in, s);
        }
        double currVal = -1;
        while (true) {
            std::string s;
            in >> s;
            std::vector<double> stamps;
            for (int i = 0; i < numbersInLine; ++i) {
                if (in >> currVal) {
                    if (i > 3) {
                        stamps.push_back(currVal);
                    }
                } else {
                    return quaternions;
                }
            }
            assert(stamps.size() == numbersInLine - 1);
            quaternions.push_back(stamps);
        }
    } else {
        std::cerr << "ERROR opening file" << std::endl;
        exit(ERROR_OPENING_FILE_READ);
    }
}

std::vector<std::vector<std::vector<double>>> gdr::getMeasurements(const std::string &pairwiseTransformations) {
    std::string currentToken;
    std::ifstream in(pairwiseTransformations);
    if (!in.is_open()) {
        exit(ERROR_OPENING_FILE_READ);
    }

    int poseCounter = 0;

    while (in >> currentToken && currentToken[0] != 'E') {
        if (currentToken[0] == 'V') {
            ++poseCounter;
        }
    }
    std::vector<std::vector<std::vector<double>>> relativePoses(poseCounter,
                                                                std::vector<std::vector<double>>(poseCounter));
    do {
        int from;
        int to;
        if (!(in >> from && in >> to)) {
            exit(ERROR_OPENING_FILE_READ);
        }

        if (from > to) {
            std::swap(from, to);
        }
        std::vector<double> currentRelativePose;
        for (int i = 0; i < 7; ++i) {
            double currentVal;
            if (!(in >> currentVal)) {
                exit(ERROR_OPENING_FILE_READ);
            }
            currentRelativePose.push_back(currentVal);
        }
        relativePoses[from][to] = currentRelativePose;

        while (true) {
            if (!(in >> currentToken)) {
                return relativePoses;
            }
            if (currentToken[0] == 'E') {
                break;
            }
        }
    } while (true);
}


std::vector<std::pair<double, double>> getErrorMeasurements(const std::string &truePairwiseTransformation,
                                                            const std::string &estimatedPairwiseTransformations) {
    return std::vector<std::pair<double, double>>();
}


