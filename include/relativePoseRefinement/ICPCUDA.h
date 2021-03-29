//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ICPCUDA_H
#define GDR_ICPCUDA_H

#include "IRefinerRelativePose.h"
#include "poseGraph/VertexCG.h"
#include <Eigen/Eigen>

namespace gdr {

    class ICPCUDA : public IRefinerRelativePose {

    public:
        bool refineRelativePose(const MatchableInfo &poseToBeTransformedICP,
                                const MatchableInfo &poseDestinationICPModel,
                                SE3 &initTransformationSE3) override;
    };
}

#endif
