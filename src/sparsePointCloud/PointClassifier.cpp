//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "sparsePointCloud/PointClassifier.h"

#include <queue>
#include <cassert>

namespace gdr {

    int PointClassifier::getPointClass(int poseNumber, int keypointIndexLocal) const {
        assert(poseNumber < pointClassesByPose.size());
        assert(poseNumber >= 0);

        const auto &classesByLocalKeyPointIndex = pointClassesByPose[poseNumber];

        auto it = classesByLocalKeyPointIndex.find(keypointIndexLocal);
        if (it != classesByLocalKeyPointIndex.end()) {
            return it->second;
        } else {
            return -1;
        }
    }

    void PointClassifier::insertPointsWithNewClasses(const std::vector<std::pair<int, int>> &points) {

        std::vector<int> insertedGlobalIndices;

        for (const auto &poseAndLocalInd: points) {
            int poseNumber = poseAndLocalInd.first;
            int localIndex = poseAndLocalInd.second;
            assert(poseNumber >= 0 && poseNumber < pointGlobalIndexByPose.size());

            int newGlobalIndex = poseNumberAndPointLocalIndexByGlobalIndex.size();
            auto foundGlobalIndex = pointGlobalIndexByPose[poseNumber].find(localIndex);

            if (foundGlobalIndex != pointGlobalIndexByPose[poseNumber].end()) {
                newGlobalIndex = foundGlobalIndex->second;
                assert(newGlobalIndex >= 0 && newGlobalIndex < poseNumberAndPointLocalIndexByGlobalIndex.size());
            } else {
                pointGlobalIndexByPose[poseNumber][localIndex] = newGlobalIndex;
                poseNumberAndPointLocalIndexByGlobalIndex.push_back(poseAndLocalInd);
            }

            int pointGlobalIndex = pointGlobalIndexByPose[poseNumber].find(localIndex)->second;
            assert(poseNumberAndPointLocalIndexByGlobalIndex[pointGlobalIndex] == poseAndLocalInd);

            insertedGlobalIndices.push_back(pointGlobalIndex);
        }

        for (int i = 0; i < insertedGlobalIndices.size(); ++i) {
            for (int j = i + 1; j < insertedGlobalIndices.size(); ++j) {

                int globalIndexI = insertedGlobalIndices[i];
                int globalIndexJ = insertedGlobalIndices[j];

                for (int to2 = 0; to2 < 2; ++to2) {
                    auto foundListOfAdjVertices = edgesBetweenPointsByGlobalIndices.find(globalIndexI);
                    if (foundListOfAdjVertices != edgesBetweenPointsByGlobalIndices.end()) {
                        foundListOfAdjVertices->second.push_back(globalIndexJ);
                    } else {
                        std::pair<int, std::vector<int>> pairToInsert = {globalIndexI, {globalIndexJ}};
                        edgesBetweenPointsByGlobalIndices.insert(pairToInsert);
                    }
                    std::swap(globalIndexJ, globalIndexI);
                }
            }
        }
    }

    int PointClassifier::getUnknownClassIndex() const {
        return unknownClassIndex;
    }

    int PointClassifier::getNumberOfPoses() const {

        assert(pointClassesByPose.size() == pointGlobalIndexByPose.size());
//        assert(matchesGlobalIndicesByPose.size() == pointGlobalIndexByPose.size());

        return pointClassesByPose.size();
    }

    int PointClassifier::getNumberOfGlobalIndices() const {

        return poseNumberAndPointLocalIndexByGlobalIndex.size();
    }

    std::vector<int> PointClassifier::assignPointClasses() {
        std::vector<bool> visitedGlobalIndices(getNumberOfGlobalIndices(), false);
        std::vector<int> classByGlobalIndex(getNumberOfGlobalIndices(), getUnknownClassIndex());

        for (int globalIndexToVisit = 0; globalIndexToVisit < getNumberOfGlobalIndices(); ++globalIndexToVisit) {

            int newClassNumber = numClasses;
            if (visitedGlobalIndices[globalIndexToVisit]) {
                continue;
            }
            //TODO: use graph libraries for BFS
            std::queue<int> globalIndicesToVisit;

            globalIndicesToVisit.push(globalIndexToVisit);
            std::vector<int> currentClassGlobalIndices;

            while (!globalIndicesToVisit.empty()) {
                int currentGlobalIndex = globalIndicesToVisit.front();
                globalIndicesToVisit.pop();
                visitedGlobalIndices[currentGlobalIndex] = true;

                classByGlobalIndex[currentGlobalIndex] = newClassNumber;
                const auto &poseAndLocalInd = poseNumberAndPointLocalIndexByGlobalIndex[currentGlobalIndex];
                assert(pointClassesByPose[poseAndLocalInd.first].find(poseAndLocalInd.second) ==
                       pointClassesByPose[poseAndLocalInd.first].end());
                pointClassesByPose[poseAndLocalInd.first][poseAndLocalInd.second] = newClassNumber;

                for (const auto &samePoint: edgesBetweenPointsByGlobalIndices[currentGlobalIndex]) {
                    if (!visitedGlobalIndices[samePoint]) {
                        globalIndicesToVisit.push(samePoint);
                        visitedGlobalIndices[samePoint] = true;
                    }
                }
            }

            ++numClasses;
        }

        for (int i = 0; i < getNumberOfGlobalIndices(); ++i) {
            assert(classByGlobalIndex[i] != getUnknownClassIndex());
            const auto &poseAndLocalInd = poseNumberAndPointLocalIndexByGlobalIndex[i];
            assert(pointClassesByPose[poseAndLocalInd.first][poseAndLocalInd.second] == classByGlobalIndex[i]);
        }
        return classByGlobalIndex;
    }

    std::pair<int, int> PointClassifier::getPoseNumberAndLocalIndex(int globalIndex) const {
        assert(globalIndex < poseNumberAndPointLocalIndexByGlobalIndex.size());
        return poseNumberAndPointLocalIndexByGlobalIndex[globalIndex];
    }

    int PointClassifier::getGetGlobalIndex(int poseNumber, int localIndex) const {

        assert(poseNumber < pointGlobalIndexByPose.size());
        auto foundIt = pointGlobalIndexByPose[poseNumber].find(localIndex);

        assert(foundIt != pointGlobalIndexByPose[poseNumber].end());
        return foundIt->second;
    }

    void PointClassifier::setNumberOfPoses(int numberOfPoses) {

        pointClassesByPose = std::vector<std::unordered_map<int, int>>(numberOfPoses);
        pointGlobalIndexByPose = std::vector<std::unordered_map<int, int>>(numberOfPoses);
    }

}