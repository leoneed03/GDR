//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINTMATCHER_H
#define GDR_POINTMATCHER_H

#include <vector>
#include <unordered_map>

namespace gdr {

    struct PointMatcher {

        int numClasses = 0;

//        std::vector<std::vector<int>> matchesGlobalIndicesByPose;
        std::vector<std::pair<int, int>> poseNumberAndPointLocalIndexByGlobalIndex;
        std::vector<std::unordered_map<int, int>> pointGlobalIndexByPose;
        std::vector<std::unordered_map<int, int>> pointClassesByPose;
        static const int unknownClassIndex = -1;

        std::pair<int, int> getPoseNumberAndLocalIndex(int globalIndex) const;
        int getGetGlobalIndex(int poseNumber, int localIndex) const;
        int getNumberOfGlobalIndices() const;
        int getNumberOfPoses() const;
        int getUnknownClassIndex() const;

        void setNumberOfPoses(int numPoses);

        std::vector<int>  assignPointClasses();

        int getPointClass(int poseNumber, int keypointIndexLocal) const;

        // point format: first -- pose number, second -- local keypoint index
        void insertPointsWithNewClasses(const std::vector<std::pair<int, int>> &pointsOneClass);
//        int insertPointsWithNewClass(const std::vector<std::pair<int, int>> &pointsOneClass);
    };
}

#endif
