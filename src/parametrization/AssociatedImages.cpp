//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <cassert>

#include "parametrization/AssociatedImages.h"

namespace gdr {
    AssociatedImages::AssociatedImages(
            const std::map<std::string, std::pair<double, std::string>> &timeAndPairedDepthByRgbToSet,
            const std::map<std::string, std::pair<double, std::string>> &timeAndPairedRgbByDepthToSet) :
            timeAndPairedRgbByDepth(timeAndPairedRgbByDepthToSet),
            timeAndPairedDepthByRgb(timeAndPairedDepthByRgbToSet) {}

    std::map<std::string, std::pair<double, std::string>> &AssociatedImages::getTimeAndPairedDepthByRgb() {
        return timeAndPairedDepthByRgb;
    }

    std::map<std::string, std::pair<double, std::string>> &AssociatedImages::getTimeAndPairedRgbByDepth() {
        return timeAndPairedRgbByDepth;
    }

    std::map<double, std::pair<std::string, std::string>>
    AssociatedImages::getMapFromDepthTimestampToRgbAndDepthFilename() const {

        std::map<double, std::pair<std::string, std::string>> timestampToRgbDepthNames;

        assert(timeAndPairedRgbByDepth.size() == timeAndPairedDepthByRgb.size());
        assert(!timeAndPairedDepthByRgb.empty());

        for (const auto &pairedInfo: timeAndPairedDepthByRgb) {
            double timeToInsert = pairedInfo.second.first;
            const std::string &depthName = pairedInfo.second.second;
            const std::string &rgbName = pairedInfo.first;

            timestampToRgbDepthNames.emplace(std::make_pair(timeToInsert,
                                                            std::make_pair(rgbName,
                                                                           depthName)));
        }

        return timestampToRgbDepthNames;
    }
}