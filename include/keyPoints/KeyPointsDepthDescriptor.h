//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTSDEPTHDESCRIPTOR_H
#define GDR_KEYPOINTSDEPTHDESCRIPTOR_H

#include "KeyPoint2DAndDepth.h"

#include <vector>

namespace gdr {

    class keyPointsDepthDescriptor {

        std::vector<KeyPoint2DAndDepth> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

    public:

        keyPointsDepthDescriptor(const std::vector<KeyPoint2DAndDepth> &keypointsKnownDepth,
                                 const std::vector<float> &descriptorsKnownDepth,
                                 const std::vector<double> &depths);

        const std::vector<KeyPoint2DAndDepth> &getKeyPointsKnownDepth() const;

        const std::vector<float> &getDescriptorsKnownDepth() const;

        const std::vector<double> &getDepths() const;

        static keyPointsDepthDescriptor filterKeypointsByKnownDepth(
                const std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>> &keypointAndDescriptor,
                const std::string &pathToDImage,
                double depthCoefficient);
    };
}

#endif
