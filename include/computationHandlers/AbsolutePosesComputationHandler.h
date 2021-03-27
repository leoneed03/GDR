//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H
#define GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H

#include <memory>
#include "poseGraph/ConnectedComponent.h"

namespace gdr {

    class AbsolutePosesComputationHandler {
        std::unique_ptr<ConnectedComponentPoseGraph> connectedComponent;
        std::unique_ptr<IPointMatcher> pointMatcher;
        std::unique_ptr<ICloudProjector> cloudProjector;

        void computePointClasses();

    public:

        explicit AbsolutePosesComputationHandler(
                std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph);

        int getNumberOfPoses() const;

        std::set<int> initialIndices() const;

        std::vector<VertexCG*> getVerticesPointers() const;

    public:

        std::vector<SO3> performRotationAveraging();

        std::vector<SO3> optimizeRotationsRobust();

        std::vector<Eigen::Vector3d> optimizeAbsoluteTranslations(int indexFixedToZero = 0);

        std::vector<SE3> performBundleAdjustmentUsingDepth(int indexFixedToZero = 0);
    };
}

#endif
