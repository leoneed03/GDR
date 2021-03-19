//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//



#ifndef GDR_IMAGEDRAWER_H
#define GDR_IMAGEDRAWER_H

#include "keyPoints/KeyPointInfo.h"

#include <vector>
#include <string>

namespace gdr {

    struct ImageDrawer {

        static int showKeyPointOnImage(const std::string &pathToRGBImage,
                                       const KeyPointInfo &keyPointInfo,
                                       int pointIndex,
                                       std::string pathToSave = "",
                                       std::string nameToSave = "");

        static int showKeyPointsOnImage(const std::string &pathToRGBImage,
                                        const std::vector<std::pair<int, KeyPointInfo>> &keyPointInfo,
                                        int maxIndexKeyPointToShow = 100,
                                        std::string pathToSave = "",
                                        std::string nameToSave = "");

        // each pair represents keyPoint's index (global class number) and info about it being a projection
        // matches: keypoints1[match.first] (on the first image) corresponds to
        // keypoints2[match.second] (on the second image)

        static int showKeyPointMatchesTwoImages(const std::string &pathToRGBImageFirst,
                                                const std::vector<KeyPointInfo> &keyPointInfosFirstImage,
                                                const std::string &pathToRGBImageSecond,
                                                const std::vector<KeyPointInfo> &keyPointInfosSecondImage,
                                                const std::vector<std::pair<int, int>> &matchesKeypoints,
                                                int maxIndexKeyPointToShow = 100,
                                                std::string pathToSave = "");
    };
}

#endif
