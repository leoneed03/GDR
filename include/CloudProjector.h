//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_CLOUDPROJECTOR_H
#define GDR_CLOUDPROJECTOR_H

#include "Point3d.h"
#include "VertexCG.h"
#include "KeyPointInfo.h"
#include "PointMatcher.h"

#include <unordered_map>
#include <vector>

namespace gdr {


    struct CloudProjector {
        int maxPointIndex = -1;
//        std::unordered_map<int, Point3d> indexedPoints;
        std::vector<Point3d> indexedPoints;
        std::vector<VertexCG *> poses;

        // i-th unordered map maps from point's index (int) to struct containing information
        // about keypoint (KeyPointInfo) -- point's observation by i-th camera
        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPose;


        // j-th vector contains pose numbers observing j-keypoint
        std::vector<std::vector<int>> numbersOfPosesObservingSpecificPoint;

//        CloudProjector() = delete;

        std::vector<Point3d> setComputedPointsGlobalCoordinates();

        void setPoses(const std::vector<VertexCG *> &cameraPoses);

        /*
         * vector contains information about which poses observe specific point
         * and Sift keypoint info like x, y, scale and orientation
         */

        int addPoint(int indexedPoint,
                     const std::vector<KeyPointInfo> &poseNumberAndProjectedKeyPointInfo);

        const Point3d &getPointByIndex3d(int pointNumber3d) const;

        const VertexCG &getPoseByPoseNumber(int poseNumber) const;

        int getPoseNumber() const;

//        KeyPointInfo& getKeyPointInfoByPoseNumberAndPointIndex(int PoseNumber, int PointIndex) const;

        std::vector<std::pair<int, KeyPointInfo>> getKeyPointsIndicesAndInfoByPose(int poseNumber) const;

        const std::vector<std::unordered_map<int, KeyPointInfo>> getKeyPointInfoByPoseNumberAndPointClass() const;

        void showPointsProjection(const std::vector<Point3d> &pointsGlobalCoordinates) const;
    };
}

#endif
