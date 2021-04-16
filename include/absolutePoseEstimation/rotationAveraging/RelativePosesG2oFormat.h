//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSESG2OFORMAT_H
#define GDR_RELATIVEPOSESG2OFORMAT_H

#include <vector>

#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"

namespace gdr {

    class RelativePosesG2oFormat {
        std::vector<RotationMeasurement> relativeRotations;
    public:
        RelativePosesG2oFormat(const std::vector<RotationMeasurement> &relativeRotationsToSet);

        friend std::ostream &operator<<(std::ostream &os, const RelativePosesG2oFormat &rotationsG2o);
    };
}


#endif
