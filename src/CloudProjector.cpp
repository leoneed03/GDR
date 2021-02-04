//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "CloudProjector.h"
#include "pointCloud.h"

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

        for (const auto &pairIndexInfo: keyPointInfoByPose[poseNumber]) {
            resultKeyPointsObserved.push_back(pairIndexInfo);
        }

        return resultKeyPointsObserved;
    }

    int CloudProjector::getPoseNumber() const {
        return poses.size();
    }

    std::vector<Point3d> CloudProjector::setComputedPointsGlobalCoordinates() {

        int pointsSize = indexedPoints.size();
        for (int i = 0; i < pointsSize; ++i) {
            assert(indexedPoints[i].getIndex() == i);
        }

        std::vector<std::vector<Eigen::Vector3d>> computedCoordinatesByPointIndex(pointsSize);

        int posesSize = poses.size();
        for (int i = 0; i < posesSize; ++i) {
            for (const auto &observedPoints: keyPointInfoByPose[i]) {

                const KeyPointInfo &currentInfoBeforeProjection = observedPoints.second;

                assert(currentInfoBeforeProjection.isInitialized());
                int currentPointIndex = observedPoints.first;

                double newX = currentInfoBeforeProjection.getX();
                double newY = currentInfoBeforeProjection.getY();
                double newZ = currentInfoBeforeProjection.getDepth();

                std::vector<double> infoBeforeProjection = {newX, newY, newZ, 1.0};

                assert(i == poses[i]->getIndex());
                assert(i == currentInfoBeforeProjection.getObservingPoseNumber());

                Eigen::Vector4d localCoordinatesBeforeProjection =
                        getPointBeforeProjection(infoBeforeProjection,
                                                 poses[i]->getCamera());
                Eigen::Vector4d globalCoordinates =
                        poses[i]->getEigenMatrixAbsolutePose4d().inverse() * localCoordinatesBeforeProjection;

                computedCoordinatesByPointIndex[currentPointIndex].push_back(globalCoordinates.block<3, 1>(0, 0));
            }
        }

        for (int i = 0; i < computedCoordinatesByPointIndex.size(); ++i) {
            Eigen::Vector3d sumCoordinates;
            sumCoordinates.setZero();
            assert(sumCoordinates.norm() < 3 * std::numeric_limits<double>::epsilon());

            for (const auto &coordinates: computedCoordinatesByPointIndex[i]) {
                sumCoordinates += coordinates;
            }
            for (int toDim = 0; toDim < sumCoordinates.rows(); ++i) {
                sumCoordinates[toDim] /= computedCoordinatesByPointIndex[i].size();
            }
            indexedPoints[i].setEigenVector3dPointXYZ(sumCoordinates);
        }


        return indexedPoints;
    }


}
