//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ICPCUDA_H
#define GDR_ICPCUDA_H

#include "RefinerRelativePose.h"

#include "keyPoints/KeyPointMatches.h"
#include "poseGraph/VertexPose.h"

#include <mutex>

namespace gdr {

    class ICPCUDA : public RefinerRelativePose {

        std::mutex deviceCudaLock;
    public:

        bool refineRelativePose(const MatchableInfo &poseToBeTransformedICP,
                                const MatchableInfo &poseDestinationICPModel,
                                const KeyPointMatches &keyPointMatches,
                                SE3 &initTransformationSE3,
                                int deviceIndex) override;
    };
}

#endif
