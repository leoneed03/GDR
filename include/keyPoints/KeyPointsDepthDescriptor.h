//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTSDEPTHDESCRIPTOR_H
#define GDR_KEYPOINTSDEPTHDESCRIPTOR_H

#include "KeyPoint2D.h"
#include <vector>

namespace gdr {

    struct keyPointsDepthDescriptor {
        std::vector<KeyPoint2D> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        keyPointsDepthDescriptor(const std::vector<KeyPoint2D> &newKeypointsKnownDepth,
                                 const std::vector<float> &newDescriptorsKnownDepth,
                                 const std::vector<double> &newDepths);

        const std::vector<KeyPoint2D> &getKeyPointsKnownDepth() const;

        const std::vector<float> &getDescriptorsKnownDepth() const;

        const std::vector<double> &getDepths() const;
    };
}
#endif
