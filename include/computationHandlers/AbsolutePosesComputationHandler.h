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

        bool printProgressToConsole = false;
        bool saveDebugImages = false;

        std::unique_ptr<ConnectedComponentPoseGraph> connectedComponent;
        std::unique_ptr<IPointClassifier> pointMatcher;
        std::unique_ptr<ICloudProjector> cloudProjector;

        void computePointClasses();

    public:

        bool getSaveDebugImages() const;

        bool getPrintProgressToCout() const;

        void setSaveDebugImages(bool saveImages);

        void setPrintProgressToCout(bool printProgress);

        explicit AbsolutePosesComputationHandler(
                std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph);

        int getNumberOfPoses() const;

        std::set<int> initialIndices() const;

        const std::vector<VertexCG> &getVertices() const;

        std::vector<SO3> performRotationAveraging();

        std::vector<SO3> performRotationRobustOptimization();

        std::vector<Eigen::Vector3d> performTranslationAveraging(int indexFixedToZero = 0);

        std::vector<SE3> performBundleAdjustmentUsingDepth(int indexFixedToZero = 0);
    };
}

#endif
