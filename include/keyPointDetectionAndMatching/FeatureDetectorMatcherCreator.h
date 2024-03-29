//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_FEATUREDETECTORMATCHERCREATOR_H
#define GDR_FEATUREDETECTORMATCHERCREATOR_H

#include <memory>
#include "FeatureDetectorMatcher.h"

namespace gdr {

    class FeatureDetectorMatcherCreator {
    public:

        FeatureDetectorMatcherCreator() = delete;

        enum class SiftDetectorMatcher {
            SIFTGPU
        };

        static std::unique_ptr<FeatureDetectorMatcher>
        getFeatureDetector(const SiftDetectorMatcher &siftDetectorMatcher);
    };
}

#endif
