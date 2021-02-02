//
// Created by leoneed on 2/1/21.
//

#include "PointMatcher.h"

#include <queue>
#include <iostream>
#include <cassert>

namespace gdr {

    void PointMatcher::setNumberOfPoses(int numPoses) {
        pointClassesByPose = std::vector<std::unordered_map<int, int>>(numPoses);
        pointGlobalIndexByPose = std::vector<std::unordered_map<int, int>>(numPoses);
//        matchesGlobalIndicesByPose = std::vector<std::vector<int>>(numPoses);
    }

    int PointMatcher::getPointClass(int poseNumber, int keypointIndexLocal) const {
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

    void PointMatcher::insertPointsWithNewClasses(const std::vector<std::pair<int, int>> &points) {

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

//        for (int i = 0; i < insertedGlobalIndices.size(); ++i) {
//            for (int j = i + 1; j < insertedGlobalIndices.size(); ++j) {
//
//                int globalIndexI = insertedGlobalIndices[i];
//                int globalIndexJ = insertedGlobalIndices[j];
//
////                int globalIndRange = poseNumberAndPointLocalIndexByGlobalIndex.size();
//                assert(globalIndexI < matchesGlobalIndicesByPose.size() &&
//                       globalIndexJ < matchesGlobalIndicesByPose.size());
//                matchesGlobalIndicesByPose[globalIndexI].push_back(globalIndexJ);
//                matchesGlobalIndicesByPose[globalIndexJ].push_back(globalIndexI);
//            }
//        }
    }

    int PointMatcher::getUnknownClassIndex() const {
        return unknownClassIndex;
    }

    int PointMatcher::getNumberOfPoses() const {

        assert(pointClassesByPose.size() == pointGlobalIndexByPose.size());
//        assert(matchesGlobalIndicesByPose.size() == pointGlobalIndexByPose.size());

        return pointClassesByPose.size();
    }

    int PointMatcher::getNumberOfGlobalIndices() const {

//        assert(!poseNumberAndPointLocalIndexByGlobalIndex.empty());
        return poseNumberAndPointLocalIndexByGlobalIndex.size();
    }

    std::vector<int> PointMatcher::assignPointClasses() {
        std::vector<bool> visitedGlobalIndices(getNumberOfGlobalIndices(), false);
        std::vector<int> classByGlobalIndex(getNumberOfGlobalIndices(), getUnknownClassIndex());

        for (int globalIndexToVisit = 0; globalIndexToVisit < getNumberOfGlobalIndices(); ++globalIndexToVisit) {

            int newClassNumber = numClasses;
            if (visitedGlobalIndices[globalIndexToVisit]) {
                continue;
            }


            std::queue<int> globalIndicesToVisit;

            globalIndicesToVisit.push(globalIndexToVisit);

            std::vector<int> currentClassGlobalIndices;

            while (!globalIndicesToVisit.empty()) {
                int currentGlobalIndex = globalIndicesToVisit.front();
                globalIndicesToVisit.pop();

                assert(!visitedGlobalIndices[currentGlobalIndex]);
                visitedGlobalIndices[currentGlobalIndex] = true;

                classByGlobalIndex[currentGlobalIndex] = newClassNumber;
                const auto &poseAndLocalInd = poseNumberAndPointLocalIndexByGlobalIndex[currentGlobalIndex];
                assert(pointClassesByPose[poseAndLocalInd.first].find(poseAndLocalInd.second) ==
                       pointClassesByPose[poseAndLocalInd.first].end());
                pointClassesByPose[poseAndLocalInd.first][poseAndLocalInd.second] = newClassNumber;
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

    std::pair<int, int> PointMatcher::getPoseNumberAndLocalIndex(int globalIndex) const {
        assert(globalIndex < poseNumberAndPointLocalIndexByGlobalIndex.size());
        return poseNumberAndPointLocalIndexByGlobalIndex[globalIndex];
    }

    int PointMatcher::getGetGlobalIndex(int poseNumber, int localIndex) const {

        assert(poseNumber < pointGlobalIndexByPose.size());
        auto foundIt = pointGlobalIndexByPose[poseNumber].find(localIndex);

        assert(foundIt != pointGlobalIndexByPose[poseNumber].end());
        return foundIt->second;
    }

}