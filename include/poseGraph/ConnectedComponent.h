//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CONNECTEDCOMPONENT_H
#define GDR_CONNECTEDCOMPONENT_H

#include "VertexCG.h"

#include "parametrization/RelativeSE3.h"
#include "parametrization/cameraRGBD.h"

#include "keyPoints/KeyPointInfo.h"

#include "sparsePointCloud/IPointClassifier.h"
#include "sparsePointCloud/ICloudProjector.h"

#include <tbb/concurrent_vector.h>
#include <vector>
#include <set>

namespace gdr {

    class ConnectedComponentPoseGraph {

        int componentNumber;
        std::vector<VertexCG> absolutePoses;

        // indexing for absolute poses is from 0 to component.size() - 1 including
        std::vector<std::vector<RelativeSE3>> relativePoses;
        CameraRGBD cameraRgbd;
        std::string relativeRotationsFile;
        std::string absoluteRotationsFile;

        /*
         * each pair is poseNumber and point's local index in pose's list of detected keypoints
         *  paired with its KeyPointInfo
         *  pairs are grouped in one vector if they represent same global point
         */

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierPointCorrespondences;

    public:

        ConnectedComponentPoseGraph(
                const std::vector<VertexCG> &absolutePoses,
                const std::vector<std::vector<RelativeSE3>> &edgesLocalIndicesRelativePoses,
                const CameraRGBD &defaultCamera,
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &inlierPointCorrespondences,
                const std::string& RelativeRotationsFile,
                const std::string& absoluteRotationsFile,
                int componentNumber);

        int getComponentNumber() const;

        std::set<int> initialIndices() const;

        std::vector<SE3> getPoses() const;

        bool poseIndexIsValid(int poseIndex) const;

    public:

        void setPoseSE3(int poseIndex, const SE3 &poseSE3);

        void setRotation(int poseIndex, const SO3 &rotationSO3);

        void setTranslation(int poseIndex, const Eigen::Vector3d &translation);

        const std::vector<RelativeSE3> &getConnectionsFromVertex(int vertexNumber) const;

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
