//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_READERTUM_H
#define GDR_READERTUM_H

#include <vector>
#include <map>

#include "parametrization/PoseFullInfo.h"

namespace gdr {

    struct AssociatedImages {
        std::map<std::string, std::pair<double, std::string>> timeAndPairedDepthByRgb;
        std::map<std::string, std::pair<double, std::string>> timeAndPairedRgbByDepth;
    };

    class ReaderTUM {
    public:
        static std::vector<PoseFullInfo>
        getPoseInfoTimeTranslationOrientation(const std::string &pathToGroundTruthFile);

        static AssociatedImages readAssocShortFilenameRgbToD(const std::string &pathToAssocFile);

        static std::vector<PoseFullInfo> getPoseInfoTimeTranslationOrientationByMatches(
                const std::vector<PoseFullInfo> &posesFullInfo,
                const std::vector<double> &timestamps,
                double timeDifTreshold
        );

        static void createRgbOrDepthTxt(const std::string &pathToImages,
                                        const std::string &pathToCreatedFileDirectory);
    };
}

#endif
