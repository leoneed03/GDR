//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "keyPointDetectionAndMatching/FeatureDetector.h"
#include "keyPointDetectionAndMatching/SiftModuleGPU.h"
#include <iostream>

namespace gdr {

    std::unique_ptr<ISiftModule> FeatureDetector::getFeatureDetector(const SiftDetectorMatcher& siftDetectorMatcher) {
        if (siftDetectorMatcher == SiftDetectorMatcher::SIFTGPU) {
            return std::make_unique<SiftModuleGPU>();
        } else {
            std::cout << "only SiftGPU is available now" << std::endl;
        }

        return std::make_unique<SiftModuleGPU>();
    }
}
