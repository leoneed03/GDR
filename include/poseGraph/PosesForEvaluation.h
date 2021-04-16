//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSESFOREVALUATION_H
#define GDR_POSESFOREVALUATION_H

#include <vector>

#include "parametrization/PoseFullInfo.h"

#include "poseGraph/VertexPose.h"

namespace gdr {

    class PosesForEvaluation {

        std::vector<PoseFullInfo> poses;

        void initFromShift(const std::vector<VertexPose> &poses, const SE3 &applyFromRight);

        void initFromShift(const std::vector<PoseFullInfo> &poses, const SE3 &applyFromRight);

    public:

        PosesForEvaluation(const std::vector<PoseFullInfo> &poses, const SE3 &applyFromRight);

        PosesForEvaluation(const std::vector<VertexPose> &poses, const SE3 &applyFromRight);

        friend std::ostream &operator<<(std::ostream &os, const PosesForEvaluation &poses);
    };
}


#endif