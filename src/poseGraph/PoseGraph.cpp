//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "poseGraph/PoseGraph.h"

namespace gdr {

    PoseGraph::PoseGraph(const std::vector<VertexCG> &poseVertices,
                         const std::vector<std::vector<RelativeSE3>> &poseRelativeFactors) :
            absolutePoses(poseVertices),
            relativePoses(poseRelativeFactors) {

        assert(absolutePoses.size() == poseRelativeFactors.size());
    }

    int PoseGraph::size() const {
        assert(absolutePoses.size() == relativePoses.size());

        return absolutePoses.size();
    }

    bool PoseGraph::empty() const {

        return size() == 0;
    }

    void PoseGraph::setCamera(int poseIndex,
                              const CameraRGBD &cameraRgbd) {
        assert(poseIndex >= 0 && poseIndex < absolutePoses.size());
        assert(absolutePoses.size() == relativePoses.size());

        absolutePoses[poseIndex].setCamera(cameraRgbd);
    }

    const VertexCG &PoseGraph::getPoseVertex(int poseIndex) const {
        assert(poseIndex >= 0 && poseIndex < absolutePoses.size());
        assert(absolutePoses.size() == relativePoses.size());

        return absolutePoses[poseIndex];
    }

    void PoseGraph::addPoseVertex(const VertexCG &poseVertex) {
        absolutePoses.emplace_back(poseVertex);

        relativePoses.push_back({});

        assert(absolutePoses.size() == relativePoses.size());
    }

    void PoseGraph::addFactorRelativePose(const RelativeSE3 &relativePose) {
        int indexFromDestination = relativePose.getIndexFrom();
        int indexToToBeTransformed = relativePose.getIndexTo();

        assert(absolutePoses.size() == relativePoses.size());

        assert(indexFromDestination >= 0 && indexFromDestination < absolutePoses.size());
        assert(indexToToBeTransformed >= 0 && indexToToBeTransformed < absolutePoses.size());

        relativePoses[indexFromDestination].emplace_back(relativePose);
    }

    const RelativeSE3 &PoseGraph::getRelativePose(int indexFromDestination, int indexInAdjacencyList) const {

        assert(absolutePoses.size() == relativePoses.size());
        assert(indexFromDestination >= 0 && indexFromDestination < absolutePoses.size());
        assert(indexInAdjacencyList >= 0 && indexInAdjacencyList < absolutePoses.size());
        assert(indexInAdjacencyList < relativePoses[indexFromDestination].size());

        return relativePoses[indexFromDestination][indexInAdjacencyList];
    }

    const std::vector<VertexCG> &PoseGraph::getPoseVertices() const {
        return absolutePoses;
    }

    const std::vector<RelativeSE3> &PoseGraph::getRelativePosesFrom(int indexFromDestination) const {
        assert(indexFromDestination >= 0 && indexFromDestination < absolutePoses.size());
        assert(absolutePoses.size() == relativePoses.size());

        return relativePoses[indexFromDestination];
    }

    int PoseGraph::getNumberOfAdjacentVertices(int indexPoseVertex) const {
        assert(indexPoseVertex >= 0 && indexPoseVertex < absolutePoses.size());
        assert(absolutePoses.size() == relativePoses.size());

        return relativePoses[indexPoseVertex].size();
    }

    void PoseGraph::setRelativePoses(const std::vector<std::vector<RelativeSE3>> &relativePosesToSet) {
        assert(relativePosesToSet.size() == relativePoses.size());
        assert(relativePosesToSet.size() == absolutePoses.size());

        relativePoses = relativePosesToSet;
    }

    void PoseGraph::setRotationSO3(int poseVertexIndex, const SO3 &orientationSO3) {
        assert(poseVertexIndex >= 0 && poseVertexIndex < absolutePoses.size());

        absolutePoses[poseVertexIndex].setRotation(orientationSO3);
    }

    void PoseGraph::setPoseSE3(int poseVertexIndex, const SE3 &poseSE3) {
        assert(poseVertexIndex >= 0 && poseVertexIndex < absolutePoses.size());

        absolutePoses[poseVertexIndex].setAbsolutePoseSE3(poseSE3);
    }

    void PoseGraph::setTranslationV3(int poseVertexIndex, const Eigen::Vector3d &translation) {
        assert(poseVertexIndex >= 0 && poseVertexIndex < absolutePoses.size());

        absolutePoses[poseVertexIndex].setTranslation(translation);
    }

    int PoseGraph::getPoseIndexWithMaxConnectivity() const {

        assert(!empty());

        int maxAdjacencyNumber = -1;

        for (int poseIndex = 0; poseIndex < absolutePoses.size(); ++poseIndex) {
            maxAdjacencyNumber = std::max(maxAdjacencyNumber, getNumberOfAdjacentVertices(poseIndex));
        }

        assert(maxAdjacencyNumber >= 0);

        return maxAdjacencyNumber;
    }

}
