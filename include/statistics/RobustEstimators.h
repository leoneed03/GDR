//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROBUSTESTIMATORS_H
#define GDR_ROBUSTESTIMATORS_H

#include <vector>

namespace gdr {

    struct RobustEstimators {
        static double getMedian(const std::vector<double> &values,
                                double quantile = 0.5);

        static double getMedianAbsoluteDeviationMultiplied(const std::vector<double> &values,
                                                           double multiplier = 1.4826);
    };
}

#endif
