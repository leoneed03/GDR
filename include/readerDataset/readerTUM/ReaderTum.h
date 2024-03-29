//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_READERTUM_H
#define GDR_READERTUM_H

#include <vector>
#include <map>

#include "parametrization/PoseFullInfo.h"
#include "parametrization/AssociatedImages.h"

#include "datasetDescriber/DatasetStructure.h"

namespace gdr {

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

        static DatasetStructure getDatasetStructure(
                const std::string &pathToDataset,
                const std::string &assocShortFilename);
    };
}

#endif
