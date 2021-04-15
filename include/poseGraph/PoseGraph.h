//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POSEGRAPH_H
#define GDR_POSEGRAPH_H

#include <vector>

#include <poseGraph/VertexCG.h>
#include <parametrization/RelativeSE3.h>

namespace gdr {

    class PoseGraph {

        std::vector<VertexCG> absolutePoses;
        std::vector<std::vector<RelativeSE3>> relativePoses;

    public:

        PoseGraph() = default;

        PoseGraph(const std::vector<VertexCG> &poseVertices,
                  const std::vector<std::vector<RelativeSE3>> &poseRelativeFactors);

        int size() const;

        bool empty() const;

        void setCamera(int poseIndex, const CameraRGBD &cameraRgbd);

        const VertexCG &getPoseVertex(int poseIndex) const;

        void addPoseVertex(const VertexCG &poseVertex);

        void addFactorRelativePose(const RelativeSE3 &relativePose);

        const RelativeSE3 &getRelativePose(int indexFromDestination, int indexInAdjacencyList) const;

        const std::vector<VertexCG> &getPoseVertices() const;

        const std::vector<RelativeSE3> &getRelativePosesFrom(int indexFromDestination) const;

        int getNumberOfAdjacentVertices(int indexPoseVertex) const;

        void setRelativePoses(const std::vector<std::vector<RelativeSE3>> &relativePosesToSet);

    };
}


#endif
