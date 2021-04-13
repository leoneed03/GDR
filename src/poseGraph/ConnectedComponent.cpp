//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "poseGraph/ConnectedComponent.h"
#include "parametrization/RelativeSE3.h"
#include "parametrization/Point3d.h"
#include "bundleAdjustment/IBundleAdjuster.h"
#include "bundleAdjustment/BundleAdjuster.h"
#include "absolutePoseEstimation/rotationAveraging/RotationRobustOptimizer.h"
#include "absolutePoseEstimation/translationAveraging/TranslationMeasurement.h"
#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"

#include <fstream>
#include <boost/filesystem.hpp>

#include "sparsePointCloud/CloudProjector.h"

namespace gdr {

    ConnectedComponentPoseGraph::ConnectedComponentPoseGraph(
            const std::vector<VertexCG> &newAbsolutePoses,
            const std::vector<std::vector<RelativeSE3>> &edgesLocalIndicesRelativePoses,
            const CameraRGBD &newDefaultCamera,
            const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &newInlierPointCorrespondences,
            const std::string &newRelativeRotationsFile,
            const std::string &newAbsoluteRotationsFile,
            int componentNumberToSet
    ) : absolutePoses(newAbsolutePoses),
        relativePoses(edgesLocalIndicesRelativePoses),
        cameraRgbd(newDefaultCamera),
        inlierPointCorrespondences(newInlierPointCorrespondences),
        relativeRotationsFile(newRelativeRotationsFile),
        absoluteRotationsFile(newAbsoluteRotationsFile),
        componentNumber(componentNumberToSet) {

        assert(getNumberOfPoses() > 0);
        assert(getNumberOfPoses() == absolutePoses.size());
    }

    int ConnectedComponentPoseGraph::getNumberOfPoses() const {
        assert(absolutePoses.size() == relativePoses.size());
        return absolutePoses.size();
    }

    std::set<int> ConnectedComponentPoseGraph::initialIndices() const {
        std::set<int> initialIndices;
        for (const auto &pose: absolutePoses) {
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
            for (int i = 0; i < relativePoses.size(); ++i) {
                for (int j = 0; j < relativePoses[i].size(); ++j) {

                    const auto &transformation = relativePoses[i][j];
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

        for (const auto &pose: absolutePoses) {
            poses.emplace_back(pose.getAbsolutePoseSE3());
        }

        assert(poses.size() == getNumberOfPoses());
        return poses;
    }

    const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &
    ConnectedComponentPoseGraph::getInlierObservedPoints() const {
        return inlierPointCorrespondences;
    }

    const VertexCG &ConnectedComponentPoseGraph::getVertex(int vertexNumber) const {
        assert(vertexNumber >= 0 && vertexNumber < absolutePoses.size());

        return absolutePoses[vertexNumber];
    }

    const std::string &ConnectedComponentPoseGraph::getPathRelativePoseFile() const {
        return relativeRotationsFile;
    }

    const std::string &ConnectedComponentPoseGraph::getPathAbsoluteRotationsFile() const {
        return absoluteRotationsFile;
    }

    void ConnectedComponentPoseGraph::setRotation(int poseIndex, const SO3 &rotationSO3) {
        assert(poseIndex >= 0 && poseIndex < absolutePoses.size());

        absolutePoses[poseIndex].setRotation(rotationSO3);
    }

    const std::vector<VertexCG> &ConnectedComponentPoseGraph::getVertices() const {
        return absolutePoses;
    }

    const std::vector<RelativeSE3> &ConnectedComponentPoseGraph::getConnectionsFromVertex(int vertexNumber) const {
        assert(poseIndexIsValid(vertexNumber));

        return relativePoses[vertexNumber];
    }

    bool ConnectedComponentPoseGraph::poseIndexIsValid(int poseIndex) const {
        assert(poseIndex < absolutePoses.size());
        assert(absolutePoses.size() == relativePoses.size());
        assert(poseIndex >= 0);

        return true;
    }

    void ConnectedComponentPoseGraph::setTranslation(int poseIndex, const Eigen::Vector3d &translation) {
        assert(poseIndexIsValid(poseIndex));

        absolutePoses[poseIndex].setTranslation(translation);
    }

    void ConnectedComponentPoseGraph::setPoseSE3(int poseIndex, const SE3 &poseSE3) {
        assert(poseIndexIsValid(poseIndex));

        absolutePoses[poseIndex].setAbsolutePoseSE3(poseSE3);
    }

    int ConnectedComponentPoseGraph::getComponentNumber() const {
        assert(componentNumber >= 0 && "component number not initialized");

        return componentNumber;
    }

    std::vector<SE3> ConnectedComponentPoseGraph::getAbsolutePoses() const {
        std::vector<SE3> absolutePosesToReturn;
        absolutePosesToReturn.reserve(getNumberOfPoses());

        for (const auto& vertex: absolutePoses) {
            absolutePosesToReturn.emplace_back(vertex.getAbsolutePoseSE3());
        }

        return absolutePosesToReturn;
    }
}