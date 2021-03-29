//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "statistics/RobustEstimators.h"

#include <algorithm>
#include <cassert>

namespace gdr {


    double RobustEstimators::getMedian(const std::vector<double> &values,
                                       double quantile) {
        std::vector<double> valuesToSort = values;

        assert(quantile >= 0.0 && quantile <= 1.0);
        int indexMedianOfMeasurements = static_cast<int>(valuesToSort.size() * quantile);

        indexMedianOfMeasurements = std::min(indexMedianOfMeasurements, static_cast<int>(valuesToSort.size() - 1));
        indexMedianOfMeasurements = std::max(0, indexMedianOfMeasurements);

        assert(indexMedianOfMeasurements >= 0 && indexMedianOfMeasurements < valuesToSort.size());

        std::nth_element(valuesToSort.begin(),
                         valuesToSort.begin() + indexMedianOfMeasurements,
                         valuesToSort.end());
        return valuesToSort[indexMedianOfMeasurements];
    }

    double RobustEstimators::getMedianAbsoluteDeviationMultiplied(const std::vector<double> &values,
                                                                  double multiplier) {

        double medianValue = getMedian(values);

        std::vector<double> residuesMedian;
        residuesMedian.reserve(values.size());

        for (const auto &value: values) {
            residuesMedian.emplace_back(std::abs(value - medianValue));
        }

        assert(residuesMedian.size() == values.size());

        double medianAbsoluteDeviation = getMedian(residuesMedian);

        return medianAbsoluteDeviation * multiplier;
    }

}