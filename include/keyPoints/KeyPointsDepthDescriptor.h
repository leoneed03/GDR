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

        keyPointsDepthDescriptor(const std::vector<KeyPoint2D> &keypointsKnownDepth,
                                 const std::vector<float> &descriptorsKnownDepth,
                                 const std::vector<double> &depths);

        const std::vector<KeyPoint2D> &getKeyPointsKnownDepth() const;

        const std::vector<float> &getDescriptorsKnownDepth() const;

        const std::vector<double> &getDepths() const;

        static keyPointsDepthDescriptor filterKeypointsByKnownDepth(
                const std::pair<std::vector<KeyPoint2D>, std::vector<float>> &keypointAndDescriptor,
                const std::string &pathToDImage);
    };
}
#endif
