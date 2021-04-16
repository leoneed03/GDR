//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSEGRAPH_H
#define GDR_POSEGRAPH_H

#include <vector>

#include <poseGraph/VertexPose.h>
#include <parametrization/RelativeSE3.h>

namespace gdr {

    class PoseGraph {

        std::vector<VertexPose> absolutePoses;
        std::vector<std::vector<RelativeSE3>> relativePoses;

    public:

        PoseGraph() = default;

        PoseGraph(const std::vector<VertexPose> &poseVertices,
                  const std::vector<std::vector<RelativeSE3>> &poseRelativeFactors);

        int size() const;

        bool empty() const;

        void setCamera(int poseIndex, const CameraRGBD &cameraRgbd);

        const VertexPose &getPoseVertex(int poseIndex) const;

        void addPoseVertex(const VertexPose &poseVertex);

        void addFactorRelativePose(const RelativeSE3 &relativePose);

        const RelativeSE3 &getRelativePose(int indexFromDestination, int indexInAdjacencyList) const;

        const std::vector<VertexPose> &getPoseVertices() const;

        const std::vector<RelativeSE3> &getRelativePosesFrom(int indexFromDestination) const;

        int getNumberOfAdjacentVertices(int indexPoseVertex) const;

        void setRelativePoses(const std::vector<std::vector<RelativeSE3>> &relativePosesToSet);

        void setRotationSO3(int poseVertexIndex, const SO3 &orientationSO3);

        void setPoseSE3(int poseVertexIndex, const SE3 &poseSE3);

        void setTranslationV3(int poseVertexIndex, const Eigen::Vector3d &translation);

        int getPoseIndexWithMaxConnectivity() const;

    };
}


#endif
