//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_REFINERRELATIVEPOSE_H
#define GDR_REFINERRELATIVEPOSE_H

#include "parametrization/MatchableInfo.h"
#include "parametrization/SE3.h"

#include "keyPoints/KeyPointMatches.h"

namespace gdr {

    /** Refine robustly estimated relative pose using depth maps */
    class RefinerRelativePose {

    public:
        /**
         * Refine SE3 relative pose between poses
         * @param poseToBeTransformedICP contains colour and depth information about
         *      pose transformed with SE3 transformation being refined
         * @param poseDestinationICPModel contains colour and depth information about destination pose
         * @param keyPointMatches contains matched keypoint indices between two colour images
         * @param initTransformationSE3 initial SE3 relative pose estimation
         * @returns true if refinement process has converged
         */
        virtual bool refineRelativePose(const MatchableInfo &poseToBeTransformed,
                                        const MatchableInfo &poseDestination,
                                        const KeyPointMatches &keyPointMatches,
                                        SE3 &initTransformationSE3) = 0;

        virtual ~RefinerRelativePose() = default;
    };
}

#endif
