//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "PosesForEvaluation.h"

namespace gdr {


    void PosesForEvaluation::initFromShift(const std::vector<VertexCG> &posesToSet, const SE3 &applyFromRight) {

        assert(!posesToSet.empty());
        for (const auto &poseToSet: posesToSet) {

            poses.emplace_back(PoseFullInfo(poseToSet.getTimestamp(), applyFromRight * poseToSet.getAbsolutePoseSE3()));
        }

        assert(posesToSet.size() == poses.size());
    }

    PosesForEvaluation::PosesForEvaluation(const std::vector<VertexCG> &posesToSet, const SE3 &applyFromRight) {

        assert(!posesToSet.empty());

        initFromShift(posesToSet, applyFromRight);
        assert(!poses.empty());
    }

    std::ostream &operator<<(std::ostream &os, const PosesForEvaluation &posesForEvaluation) {

        for (const auto &pose: posesForEvaluation.poses) {
            os << pose << std::endl;
        }

    }

    void
    PosesForEvaluation::initFromShift(const std::vector<PoseFullInfo> &posesToSet, const SE3 &applyFromRight) {

        poses = posesToSet;

        for (auto &pose: poses) {
            pose = PoseFullInfo(pose.getTimestamp(), applyFromRight * pose.poseSE3);
        }
    }

    PosesForEvaluation::PosesForEvaluation(const std::vector<PoseFullInfo> &poses,
                                           const SE3 &applyFromRight) {
        initFromShift(poses, applyFromRight);
    }
}