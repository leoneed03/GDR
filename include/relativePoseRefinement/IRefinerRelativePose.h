//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IREFINERRELATIVEPOSE_H
#define GDR_IREFINERRELATIVEPOSE_H

#include "parametrization/MatchableInfo.h"
#include "parametrization/SE3.h"

namespace gdr {

    class IRefinerRelativePose {
    public:
        virtual bool refineRelativePose(const MatchableInfo &poseToBeTransformed,
                                        const MatchableInfo &poseDestination,
                                        SE3 &initTransformationSE3) = 0;

        virtual ~IRefinerRelativePose() = default;
    };
}

#endif
