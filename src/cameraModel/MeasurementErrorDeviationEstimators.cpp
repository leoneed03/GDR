//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "cameraModel/MeasurementErrorDeviationEstimators.h"

namespace gdr {

    MeasurementErrorDeviationEstimators::MeasurementErrorDeviationEstimators(
            const std::function<double(double, double)> &dividerReprojectionErrorEstimator,
            const std::function<double(double, double)> &dividerDepthErrorEstimator) :
            dividerReprojectionError(dividerReprojectionErrorEstimator),
            dividerDepthError(dividerDepthErrorEstimator) {}

    const std::function<double(double, double)> &MeasurementErrorDeviationEstimators::getDividerDepthErrorEstimator() const {
        return dividerDepthError;
    }

    const std::function<double(double, double)> &MeasurementErrorDeviationEstimators::getDividerReprojectionEstimator() const {
        return dividerReprojectionError;
    }

    void MeasurementErrorDeviationEstimators::setParameterNoiseModelReprojection(double paramReproj) {
        defaultLinearParameterNoiseModelReprojectionBA = paramReproj;
    }

    double MeasurementErrorDeviationEstimators::getParameterNoiseModelReprojection() const {
        return defaultLinearParameterNoiseModelReprojectionBA;
    }

    void MeasurementErrorDeviationEstimators::setParameterNoiseModelDepth(double paramDepth) {
        defaultQuadraticParameterNoiseModelDepth = paramDepth;
    }

    double MeasurementErrorDeviationEstimators::getParameterNoiseModelDepth() const {
        return defaultQuadraticParameterNoiseModelDepth;
    }
}