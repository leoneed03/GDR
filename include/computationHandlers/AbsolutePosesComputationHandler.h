//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H
#define GDR_ABSOLUTEPOSESCOMPUTATIONHANDLER_H

#include <chrono>
#include <memory>

#include "absolutePoseEstimation/rotationAveraging/RotationMeasurement.h"

#include "poseGraph/ConnectedComponent.h"

namespace gdr {

    class AbsolutePosesComputationHandler {


    public:
        std::chrono::high_resolution_clock::time_point timeStartRotationAveraging;
        std::chrono::high_resolution_clock::time_point timeStartRobustRotationOptimization;
        std::chrono::high_resolution_clock::time_point timeStartTranslationAveraging;
        std::chrono::high_resolution_clock::time_point timeStartBundleAdjustment;


        std::chrono::high_resolution_clock::time_point timeEndRotationAveraging;
        std::chrono::high_resolution_clock::time_point timeEndRobustRotationOptimization;
        std::chrono::high_resolution_clock::time_point timeEndTranslationAveraging;
        std::chrono::high_resolution_clock::time_point timeEndBundleAdjustment;

    private:
        bool saveDebugImages = false;

        std::unique_ptr<ConnectedComponentPoseGraph> connectedComponent;
        std::unique_ptr<PointClassifier> pointMatcher;
        std::unique_ptr<CloudProjector> cloudProjector;

        std::string pathRelativePosesFile = "relativePosesG2o.txt";

        void computePointClasses();

    public:

        std::string getPathRelativePoseFile() const;

        bool getSaveDebugImages() const;

        void setSaveDebugImages(bool saveImages);

        explicit AbsolutePosesComputationHandler(
                std::unique_ptr<ConnectedComponentPoseGraph> &connectedComponentPoseGraph);

        void setRelativePosesFilePath(const std::string &relativePosesPathToSet);

        int getNumberOfPoses() const;

        std::set<int> initialIndices() const;

        const std::vector<VertexPose> &getVertices() const;

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

        std::stringstream printTimeBenchmarkInfo() const;
    };
}

#endif
