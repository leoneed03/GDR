//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSEESTIMATION_H
#define GDR_POSEESTIMATION_H

#include <vector>
#include <string>
#include <map>

namespace gdr {

    struct errorStats {
        double meanError = 0;
        double standartDeviation = 0;

        errorStats(double mse, double msd);
    };

    std::pair<errorStats, errorStats> getErrorStatsTranslationRotation(
            const std::vector<std::map<int, std::pair<double, double>>> &errorRelativePosesTranslationRotation);

//    std::pair<errorStats, errorStats> getErrorStatsTranslationRotationFromGroundTruthAndEstimatedPairWise(
//            const std::string &pathToGroundTruth, const std::string &estimatedPairWise);

    std::vector<std::map<int, std::pair<double, double>>>
    getErrorsTranslationRotation(const std::vector<std::vector<std::vector<double>>> &truePairWiseTransformations,
                                 const std::vector<std::vector<std::vector<double>>> &estimatedPairWiseTransformations);
}

#endif
