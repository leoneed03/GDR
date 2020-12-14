//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/files.h"
#include <algorithm>
#include <fstream>
#include <Eigen/Eigen>

std::vector<std::string> readRgbData(std::string pathToRGB) {
    DIR *pDIR;
    struct dirent *entry;
    std::vector<std::string> RgbImages;
    std::cout << "start reading" << std::endl;
    if ((pDIR = opendir(pathToRGB.data())) != nullptr) {
        int imageCounter = 0;
        while ((entry = readdir(pDIR)) != nullptr) {
            if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
                continue;
            }
            std::string absolutePathToRgbImage = pathToRGB + "/" + entry->d_name;
            RgbImages.emplace_back(absolutePathToRgbImage);
        }
        closedir(pDIR);
    } else {
        std::cout << "Unable to open" << std::endl;
    }
    std::sort(RgbImages.begin(), RgbImages.end());
    for (int i = 0; i < RgbImages.size(); ++i) {
        std::cout << i + 1 << "::" << RgbImages[i] << std::endl;
    }
    return RgbImages;
}

std::vector<std::vector<double>> parseAbsoluteRotationsFile(const std::string &pathToRotationsFile) {
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
        std::cout << "ERROR opening file" << std::endl;
    }
    return quaternions;
}
