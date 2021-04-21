//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ASSOCIATEDIMAGES_H
#define GDR_ASSOCIATEDIMAGES_H

#include <string>
#include <map>

namespace gdr {

    class AssociatedImages {
        std::map<std::string, std::pair<double, std::string>> timeAndPairedDepthByRgb;
        std::map<std::string, std::pair<double, std::string>> timeAndPairedRgbByDepth;

    public:
        AssociatedImages() = default;

        AssociatedImages(
                const std::map<std::string, std::pair<double, std::string>> &timeAndPairedDepthByRgb,
                const std::map<std::string, std::pair<double, std::string>> &timeAndPairedRgbByDepth);


        std::map<std::string, std::pair<double, std::string>> &getTimeAndPairedDepthByRgb();

        std::map<std::string, std::pair<double, std::string>> &getTimeAndPairedRgbByDepth();

        std::map<double, std::pair<std::string, std::string>> getMapFromDepthTimestampToRgbAndDepthFilename() const;
    };
}


#endif
