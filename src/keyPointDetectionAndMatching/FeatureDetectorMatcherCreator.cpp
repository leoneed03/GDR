//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.h"
#include "keyPointDetectionAndMatching/SiftModuleGPU.h"
#include <iostream>

namespace gdr {

    std::unique_ptr<IFeatureDetectorMatcher>
    FeatureDetectorMatcherCreator::getFeatureDetector(const SiftDetectorMatcher &siftDetectorMatcher) {

        if (siftDetectorMatcher == SiftDetectorMatcher::SIFTGPU) {
        } else {
            std::cout << "only SiftGPU is available now" << std::endl;
        }

        return std::make_unique<SiftModuleGPU>();
    }
}
