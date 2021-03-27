//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CONNECTEDCOMPONENT_H
#define GDR_CONNECTEDCOMPONENT_H

#include "VertexCG.h"
#include "parametrization/RelativePoseSE3.h"
#include "parametrization/cameraRGBD.h"
#include "keyPoints/KeyPointInfo.h"
#include "sparsePointCloud/IPointMatcher.h"
#include "sparsePointCloud/ICloudProjector.h"

#include <tbb/concurrent_vector.h>
#include <vector>
#include <set>

namespace gdr {

    struct ConnectedComponentPoseGraph {


        int componentGlobalNumberOptional;
        std::vector<VertexCG> absolutePoses;
//        std::vector<int> initialPosesIndices;

        // indexing for absolute poses is from 0 to component.size() - 1 including
        std::vector<std::vector<RelativePoseSE3>> relativePoses;
        std::unique_ptr<IPointMatcher> pointMatcher;
        std::unique_ptr<ICloudProjector> cloudProjector;
        CameraRGBD cameraRgbd;
        std::string relativeRotationsFile;
        std::string absoluteRotationsFile;

        /*
         * each pair is poseNumber and point's local index in pose's list of detected keypoints
         *  paired with its KeyPointInfo
         *  pairs are grouped in one vector if representing same global point
         */

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierPointCorrespondences;

        ConnectedComponentPoseGraph(
                const std::vector<VertexCG> &absolutePoses,
                const std::vector<std::vector<RelativePoseSE3>> &edgesLocalIndicesRelativePoses,
                const CameraRGBD &defaultCamera,
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &inlierPointCorrespondences,
                const std::string& RelativeRotationsFile,
                const std::string& absoluteRotationsFile,
                int componentNumber = -1);

    public:

        std::vector<Eigen::Matrix4d> getAbsolutePosesEigenMatrix4d() const;

        std::set<int> initialIndices() const;

        std::vector<VertexCG*> getVerticesPointers();

        int size() const;

        std::vector<SE3> getPoses() const;

        bool poseIndexIsValid(int poseIndex) const;

    public:

        void setPoseSE3(int poseIndex, const SE3 &poseSE3);

        void setRotation(int poseIndex, const SO3 &rotationSO3);

        void setTranslation(int poseIndex, const Eigen::Vector3d &translation);

        const std::vector<RelativePoseSE3> &getConnectionsFromVertex(int vertexNumber) const;

        const std::vector<VertexCG> &getVertices() const;

        const std::string &getPathRelativePoseFile() const;

        const std::string &getPathAbsoluteRotationsFile() const;

        const VertexCG &getVertex(int vertexNumber) const;

        int getNumberOfPoses() const;

        const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &getInlierObservedPoints() const;

    public:

        int printRelativeRotationsToFile(const std::string& pathToFileRelativeRotations) const;
    };

}

#endif
