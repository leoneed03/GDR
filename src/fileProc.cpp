//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "fileProc.h"
#include "printer.h"

#include <algorithm>
#include <fstream>

namespace gdr {

    std::vector<std::string> readRgbData(std::string pathToRGB) {
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
        }
        std::sort(RgbImages.begin(), RgbImages.end());
        return RgbImages;
    }

    std::vector<std::vector<double>> parseAbsoluteRotationsFile(const std::string &pathToRotationsFile) {
        std::ifstream in(pathToRotationsFile);
        std::vector<std::vector<double>> quaternions;
        int numOfEmptyLines = 0;
        int numbersInLine = 8;

        if (in.is_open()) {
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
        }
        return quaternions;
    }

    std::vector<std::vector<std::vector<double>>> getMeasurements(const std::string &pairwiseTransformations) {
        std::string currentToken;
        std::ifstream in(pairwiseTransformations);
        if (!in.is_open()) {
            return std::vector<std::vector<std::vector<double>>>();
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
            if (!(in >> to && in >> from)) {
                return relativePoses;
            }

            assert(to > from);
            std::vector<double> currentRelativePose;
            for (int i = 0; i < 7; ++i) {
                double currentVal;
                if (!(in >> currentVal)) {
                    return relativePoses;
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
}
