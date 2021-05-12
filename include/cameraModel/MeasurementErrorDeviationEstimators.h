//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_MEASUREMENTERRORDEVIATIONESTIMATORS_H
#define GDR_MEASUREMENTERRORDEVIATIONESTIMATORS_H

#include <functional>
#include <cmath>

namespace gdr {

    class MeasurementErrorDeviationEstimators {


        double defaultQuadraticParameterNoiseModelDepth = 3.331e-3;
        double defaultLinearParameterNoiseModelReprojectionBA = 0.987;

        std::function<double(double, double)> dividerReprojectionError =
                [](double scale, double linearParameterNoiseModelReprojection) {
                    return linearParameterNoiseModelReprojection * scale;
                };

        std::function<double(double, double)> dividerDepthError =
                [](double depth, double quadraticParameterNoiseModelDepth) {
                    double squaredDepth = std::pow(depth, 2.0);
                    return quadraticParameterNoiseModelDepth * squaredDepth;
                };

    public:
        MeasurementErrorDeviationEstimators() = default;

        MeasurementErrorDeviationEstimators(
                const std::function<double(double, double)> &dividerReprojectionErrorEstimator,
                const std::function<double(double, double)> &dividerDepthErrorEstimator);

        const std::function<double(double, double)> &getDividerDepthErrorEstimator() const;

        const std::function<double(double, double)> &getDividerReprojectionEstimator() const;

        void setParameterNoiseModelReprojection(double paramReproj);

        double getParameterNoiseModelReprojection() const;

        void setParameterNoiseModelDepth(double paramDepth);

        double getParameterNoiseModelDepth() const;
    };
}


#endif
