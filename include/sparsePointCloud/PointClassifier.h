//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINTCLASSIFIER_H
#define GDR_POINTCLASSIFIER_H

#include <vector>
#include <unordered_map>

#include "sparsePointCloud/IPointClassifier.h"

namespace gdr {

    class PointClassifier : public IPointClassifier {

    public:
        void setNumberOfPoses(int numberOfPoses) override;

        std::vector<int> assignPointClasses() override;

        void insertPointsWithNewClasses(const std::vector<std::pair<int, int>> &pointsOneClass) override;

        std::pair<int, int> getPoseNumberAndLocalIndex(int globalIndex) const override;

        int getNumberOfPoses() const override;

    private:
        int numClasses = 0;
        std::unordered_map<int, std::vector<int>> edgesBetweenPointsByGlobalIndices;
        std::vector<std::pair<int, int>> poseNumberAndPointLocalIndexByGlobalIndex;
        std::vector<std::unordered_map<int, int>> pointGlobalIndexByPose;
        std::vector<std::unordered_map<int, int>> pointClassesByPose;

        static const int unknownClassIndex = -1;

        int getGetGlobalIndex(int poseNumber, int localIndex) const;

        int getNumberOfGlobalIndices() const;

        int getUnknownClassIndex() const;

        int getPointClass(int poseNumber, int keypointIndexLocal) const;
    };
}

#endif
