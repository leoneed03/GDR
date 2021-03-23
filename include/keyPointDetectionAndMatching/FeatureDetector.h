//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_FEATUREDETECTOR_H
#define GDR_FEATUREDETECTOR_H

#include <memory>
#include "ISiftModule.h"

namespace gdr {
    class FeatureDetector {

        FeatureDetector() = delete;

    public:

        enum class SiftDetectorMatcher {SIFTGPU, SIFTCPU};

        static std::unique_ptr<ISiftModule> getFeatureDetector(const SiftDetectorMatcher& siftDetectorMatcher);
    };
}

#endif
