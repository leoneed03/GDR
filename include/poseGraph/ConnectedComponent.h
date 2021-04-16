//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CONNECTEDCOMPONENT_H
#define GDR_CONNECTEDCOMPONENT_H

#include "VertexPose.h"
#include "poseGraph/PoseGraph.h"
#include "poseGraph/PosesForEvaluation.h"

#include "parametrization/RelativeSE3.h"
#include "cameraModel/CameraRGBD.h"

#include "keyPoints/KeyPointInfo.h"

#include "sparsePointCloud/IPointClassifier.h"
#include "sparsePointCloud/ICloudProjector.h"

#include "keyPoints/KeyPointMatches.h"
#include <vector>

namespace gdr {

    class ConnectedComponentPoseGraph {

        int componentNumber;

        PoseGraph poseGraph;

        KeyPointMatches inlierPointCorrespondences;

    public:

        ConnectedComponentPoseGraph(
                const std::vector<VertexPose> &absolutePoses,
                const std::vector<std::vector<RelativeSE3>> &edgesLocalIndicesRelativePoses,
                const KeyPointMatches &inlierPointCorrespondences,
                int componentNumber);

        int getComponentNumber() const;

        std::set<int> initialIndices() const;

        std::vector<SE3> getPoses() const;

        bool poseIndexIsValid(int poseIndex) const;

        void setPoseSE3(int poseIndex, const SE3 &poseSE3);

        void setRotation(int poseIndex, const SO3 &rotationSO3);

        void setTranslation(int poseIndex, const Eigen::Vector3d &translation);

        std::vector<SE3> getAbsolutePoses() const;

        const std::vector<RelativeSE3> &getConnectionsFromVertex(int vertexNumber) const;

        const std::vector<VertexPose> &getVertices() const;

        const VertexPose &getVertex(int vertexNumber) const;

        int getNumberOfPoses() const;

        const KeyPointMatches &getInlierObservedPoints() const;

        int printRelativeRotationsToFile(const std::string &pathToFileRelativeRotations) const;

        int getPoseIndexWithMaxConnectivity() const;

        const PoseGraph &getPoseGraph() const;

        PosesForEvaluation getPosesForEvaluation() const;

        PosesForEvaluation getPosesForEvaluation(const SE3 &poseFixedZero) const;
    };

}

#endif
