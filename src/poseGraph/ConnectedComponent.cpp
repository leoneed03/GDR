//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "poseGraph/ConnectedComponent.h"
#include "parametrization/RelativeSE3.h"
#include "parametrization/Point3d.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizerLogSO3.h"
#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"

#include <fstream>
#include <boost/filesystem.hpp>

namespace gdr {

    ConnectedComponentPoseGraph::ConnectedComponentPoseGraph(
            const std::vector<VertexPose> &absolutePosesToSet,
            const std::vector<std::vector<RelativeSE3>> &edgesLocalIndicesRelativePoses,
            const KeyPointMatches &inlierPointCorrespondencesToSet,
            int componentNumberToSet) :
            inlierPointCorrespondences(inlierPointCorrespondencesToSet),
            componentNumber(componentNumberToSet) {

        poseGraph = PoseGraph(absolutePosesToSet, edgesLocalIndicesRelativePoses);
        assert(getNumberOfPoses() > 0);
    }

    int ConnectedComponentPoseGraph::getNumberOfPoses() const {
        return poseGraph.size();
    }

    std::set<int> ConnectedComponentPoseGraph::initialIndices() const {
        std::set<int> initialIndices;
        for (const auto &pose: poseGraph.getPoseVertices()) {
            initialIndices.insert(pose.getInitialIndex());
        }
        return initialIndices;
    }


    int
    ConnectedComponentPoseGraph::printRelativeRotationsToFile(const std::string &pathToFileRelativeRotations) const {

        std::ofstream file(pathToFileRelativeRotations);

        if (file.is_open()) {

            int numPoses = getNumberOfPoses();

            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }

            for (int i = 0; i < poseGraph.size(); ++i) {
                for (int j = 0; j < poseGraph.getNumberOfAdjacentVertices(i); ++j) {

                    const auto &transformation = poseGraph.getRelativePose(i, j);
                    if (i >= transformation.getIndexTo()) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = transformation.getIndexTo();
                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)(gtsam format)
                    file << "EDGE_SE3:QUAT " << indexFrom << ' ' << indexTo << ' ';
                    auto translationVector = transformation.getRelativeTranslation();
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';

                    const auto &qR = transformation.getRelativeRotation();
                    file << qR << noise << '\n';
                }
            }
        } else {
            return 1;
        }

        return 0;
    }


    std::vector<SE3> ConnectedComponentPoseGraph::getPoses() const {
        std::vector<SE3> poses;
        poses.reserve(getNumberOfPoses());

        for (const auto &pose: poseGraph.getPoseVertices()) {
            poses.emplace_back(pose.getAbsolutePoseSE3());
        }

        assert(poses.size() == getNumberOfPoses());
        return poses;
    }

    const KeyPointMatches &
    ConnectedComponentPoseGraph::getInlierObservedPoints() const {
        return inlierPointCorrespondences;
    }

    const VertexPose &ConnectedComponentPoseGraph::getVertex(int vertexNumber) const {
        return poseGraph.getPoseVertex(vertexNumber);
    }

    void ConnectedComponentPoseGraph::setRotation(int poseIndex, const SO3 &rotationSO3) {
        poseGraph.setRotationSO3(poseIndex, rotationSO3);
    }

    const std::vector<VertexPose> &ConnectedComponentPoseGraph::getVertices() const {
        return poseGraph.getPoseVertices();
    }

    const std::vector<RelativeSE3> &ConnectedComponentPoseGraph::getConnectionsFromVertex(int vertexNumber) const {

        return poseGraph.getRelativePosesFrom(vertexNumber);
    }

    bool ConnectedComponentPoseGraph::poseIndexIsValid(int poseIndex) const {
        bool isValid = poseIndex < poseGraph.size() && poseIndex >= 0;

        assert(isValid);
        return true;
    }

    void ConnectedComponentPoseGraph::setTranslation(int poseIndex, const Eigen::Vector3d &translation) {
        poseGraph.setTranslationV3(poseIndex, translation);
    }

    void ConnectedComponentPoseGraph::setPoseSE3(int poseIndex, const SE3 &poseSE3) {
        poseGraph.setPoseSE3(poseIndex, poseSE3);
    }

    int ConnectedComponentPoseGraph::getComponentNumber() const {
        assert(componentNumber >= 0 && "component number not initialized");

        return componentNumber;
    }

    std::vector<SE3> ConnectedComponentPoseGraph::getAbsolutePoses() const {
        std::vector<SE3> absolutePosesToReturn;
        absolutePosesToReturn.reserve(getNumberOfPoses());

        for (const auto &vertex: poseGraph.getPoseVertices()) {
            absolutePosesToReturn.emplace_back(vertex.getAbsolutePoseSE3());
        }

        return absolutePosesToReturn;
    }


    int ConnectedComponentPoseGraph::getPoseIndexWithMaxConnectivity() const {
        return poseGraph.getPoseIndexWithMaxConnectivity();
    }

    const PoseGraph &ConnectedComponentPoseGraph::getPoseGraph() const {
        return poseGraph;
    }

    PosesForEvaluation ConnectedComponentPoseGraph::getPosesForEvaluation() const {

        const auto &fixedToZeroPose = poseGraph.getPoseVertex(poseGraph.getPoseIndexWithMaxConnectivity());

        assert(!getVertices().empty());

        return getPosesForEvaluation(SE3(fixedToZeroPose.getAbsolutePoseSE3().inverse()));
    }

    PosesForEvaluation ConnectedComponentPoseGraph::getPosesForEvaluation(const SE3 &poseFixedZeroInversed) const {

        return PosesForEvaluation(getVertices(), poseFixedZeroInversed);
    }
}