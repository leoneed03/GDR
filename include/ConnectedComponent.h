//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CONNECTEDCOMPONENT_H
#define GDR_CONNECTEDCOMPONENT_H

#include "VertexCG.h"
#include "parametrization/RelativePoseSE3.h"
#include "parametrization/cameraRGBD.h"
#include "KeyPointInfo.h"
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

        // indexing for absolute poses is from 0 to component.size() - 1 incl.
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

        int getNumberOfPoses() const;

        void computePointClasses();

        std::vector<Eigen::Quaterniond> performRotationAveraging();

        std::vector<Eigen::Quaterniond> optimizeRotationsRobust();

        std::vector<Eigen::Vector3d> optimizeAbsoluteTranslations(int indexFixedToZero = 0);

        std::vector<SE3> performBundleAdjustmentUsingDepth(int indexFixedToZero = 0);

        std::vector<Eigen::Matrix4d> getAbsolutePosesEigenMatrix4d() const;

        std::set<int> initialIndices() const;

        int printRelativeRotationsToFile(const std::string& pathToFileRelativeRotations) const;

        std::vector<VertexCG*> getVerticesPointers();

        int size() const;
    };

}

#endif
