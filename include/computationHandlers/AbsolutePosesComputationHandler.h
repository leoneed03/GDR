//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H
#define GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H

#include <memory>

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

#include "poseGraph/ConnectedComponent.h"

namespace gdr {

    class AbsolutePosesComputationHandler {

        bool printProgressToConsole = false;
        bool saveDebugImages = false;

        std::unique_ptr<ConnectedComponentPoseGraph> connectedComponent;
        std::unique_ptr<IPointClassifier> pointMatcher;
        std::unique_ptr<ICloudProjector> cloudProjector;

        std::string pathRelativePosesFile = "relativePosesG2o.txt";

        void computePointClasses();

    public:

        std::string getPathRelativePoseFile() const;

        bool getSaveDebugImages() const;

        bool getPrintProgressToCout() const;

        void setSaveDebugImages(bool saveImages);

        void setPrintProgressToCout(bool printProgress);

        explicit AbsolutePosesComputationHandler(
                std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph);

        void setRelativePosesFilePath(const std::string &relativePosesPathToSet);

        int getNumberOfPoses() const;

        std::set<int> initialIndices() const;

        const std::vector<VertexCG> &getVertices() const;

        std::vector<double> getPosesTimestamps() const;

        std::vector<SO3> performRotationAveraging();

        std::vector<SO3> performRotationRobustOptimization();

        std::vector<Eigen::Vector3d> performTranslationAveraging();

        std::vector<SE3> performBundleAdjustmentUsingDepth();

        std::vector<SE3> getPosesSE3() const;

        const PoseGraph &getPoseGraph() const;

        int getIndexFixedPose() const;

        PosesForEvaluation getPosesForEvaluation() const;

        PosesForEvaluation getPosesForEvaluation(const SE3 &fixedPose) const;

        std::vector<RotationMeasurement> getRelativeRotationsVector() const;
    };
}

#endif
