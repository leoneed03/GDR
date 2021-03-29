//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTSANDDESCRIPTORS_H
#define GDR_KEYPOINTSANDDESCRIPTORS_H

#include <vector>
#include "keyPoints/KeyPoint2DAndDepth.h"

namespace gdr {

    struct KeyPointsDescriptors {
    private:
        std::vector<KeyPoint2DAndDepth> keypoints;
        std::vector<float> descriptors;
        std::vector<double> depths;
    public:
        KeyPointsDescriptors(
                const std::vector<KeyPoint2DAndDepth> &keypoints,
                const std::vector<float> &descriptors,
                const std::vector<double> &depths);

        const std::vector<KeyPoint2DAndDepth>& getKeyPoints() const;

        const std::vector<float>& getDescriptors() const;

        const std::vector<double>& getDepths() const;

    };
}

#endif
