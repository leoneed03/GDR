//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <cassert>

#include "keyPointDetectionAndMatching/KeyPointsAndDescriptors.h"

namespace gdr {

    KeyPointsDescriptors::KeyPointsDescriptors(const std::vector<KeyPoint2DAndDepth> &keypointsToSet,
                                               const std::vector<float> &descriptorsToSet,
                                               const std::vector<double> &depthsToSet) :
            keypoints(keypointsToSet),
            descriptors(descriptorsToSet),
            depths(depthsToSet) {
        assert(128 * keypoints.size() == descriptors.size());
        assert(depths.size() == keypoints.size());
    }

    const std::vector<KeyPoint2DAndDepth> &KeyPointsDescriptors::getKeyPoints() const {
        return keypoints;
    }

    const std::vector<float> &KeyPointsDescriptors::getDescriptors() const {
        return descriptors;
    }

    const std::vector<double> &KeyPointsDescriptors::getDepths() const {
        return depths;
    }
}