//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTSDEPTHDESCRIPTOR_H
#define GDR_KEYPOINTSDEPTHDESCRIPTOR_H

#include "SiftGPU.h"
#include <vector>

namespace gdr {

    struct keyPointsDepthDescriptor {
        std::vector<SiftGPU::SiftKeypoint> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        keyPointsDepthDescriptor(const std::vector<SiftGPU::SiftKeypoint> &newKeypointsKnownDepth,
                                 const std::vector<float> &newDescriptorsKnownDepth,
                                 const std::vector<double> &newDepths);

        const std::vector<SiftGPU::SiftKeypoint> &getKeyPointsKnownDepth() const;

        const std::vector<float> &getDescriptorsKnownDepth() const;

        const std::vector<double> &getDepths() const;
    };
}
#endif
