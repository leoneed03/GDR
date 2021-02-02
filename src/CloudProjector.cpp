//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CloudProjector.h"

namespace gdr {

    void CloudProjector::setPoses(const std::vector<VertexCG *> &cameraPoses) {
        poses = cameraPoses;
        keyPointInfoByPose = std::vector<std::unordered_map<int, KeyPointInfo>>(cameraPoses.size());
    }

    int CloudProjector::addPoint(int pointIndex,
                                 const std::vector<KeyPointInfo> &poseNumbersAndProjectedKeyPointInfo) {

        assert(poses.size() > 0);
        maxPointIndex = std::max(pointIndex, maxPointIndex);


        for (const auto &poseAndInfo: poseNumbersAndProjectedKeyPointInfo) {
            int poseNumber = poseAndInfo.getObservingPoseNumber();
            assert(poseNumber >= 0);
            const KeyPointInfo &info = poseAndInfo;

            assert(poses.size() == keyPointInfoByPose.size());
            assert(poseNumber < keyPointInfoByPose.size() && poseNumber >= 0);
            keyPointInfoByPose[poseNumber].insert(std::make_pair(pointIndex, info));

        }

        return 0;
    }

    const Point3d &CloudProjector::getPointByIndex3d(int pointNumber3d) const {
        assert(pointNumber3d >= 0 && pointNumber3d < indexedPoints.size());
        return indexedPoints[pointNumber3d];
    }

    const VertexCG &CloudProjector::getPoseByPoseNumber(int poseNumber) const {
        assert(poseNumber >= 0 && poseNumber < poses.size());
        return *poses[poseNumber];
    }

    std::vector<std::pair<int, KeyPointInfo>> CloudProjector::getKeyPointsIndicesAndInfoByPose(int poseNumber) const {

        assert(poseNumber >= 0 && poseNumber < poses.size());

        assert(poseNumber < keyPointInfoByPose.size());

        std::vector<std::pair<int, KeyPointInfo>> resultKeyPointsObserved;

        for (const auto& pairIndexInfo: keyPointInfoByPose[poseNumber]) {
            resultKeyPointsObserved.push_back(pairIndexInfo);
        }

        return resultKeyPointsObserved;
    }

    int CloudProjector::getPoseNumber() const {
        return poses.size();
    }


}
