#include "../include/files.h"
#include <algorithm>

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
//            std::cout << ++imageCounter << ": " << absolutePathToRgbImage << "\n";
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
