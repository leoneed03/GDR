//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSESCOMPUTATIONHANDLER_H
#define GDR_RELATIVEPOSESCOMPUTATIONHANDLER_H

#include "computationHandlers/AbsolutePosesComputationHandler.h"

#include "poseGraph/graphAlgorithms/GraphTraverser.h"
#include "poseGraph/CorrespondenceGraph.h"

#include "relativePoseEstimators/InlierCounter.h"

#include "datasetDescriber/DatasetCameraDescriber.h"

#include "computationHandlers/ThreadPoolTBB.h"

#include "datasetDescriber/DatasetStructure.h"

#include <chrono>

namespace gdr {

    class RelativePosesComputationHandler {

    public:
        std::chrono::high_resolution_clock::time_point timeStartDescriptors;
        std::chrono::high_resolution_clock::time_point timeStartMatching;
        std::chrono::high_resolution_clock::time_point timeStartRelativePoses;


        std::chrono::high_resolution_clock::time_point timeEndDescriptors;
        std::chrono::high_resolution_clock::time_point timeEndMatching;
        std::chrono::high_resolution_clock::time_point timeEndRelativePoses;

    private:
        bool printInformationConsole = false;
        int numberOfThreadsCPU = 1;
        int deviceCudaICP = 0;

        std::vector<std::pair<double, double>> timestampsRgbDepthAssociated;
        std::unique_ptr<FeatureDetectorMatcher> siftModule;
        std::unique_ptr<EstimatorRelativePoseRobust> relativePoseEstimatorRobust;
        std::unique_ptr<RefinerRelativePose> relativePoseRefiner;
        // unused right now
        ThreadPoolTBB threadPool;
        CameraRGBD cameraDefault;

        std::vector<CameraRGBD> camerasRgbByPoseIndex;
        std::vector<CameraRGBD> camerasDepthByPoseIndex;

        ParamsRANSAC paramsRansac;
        InlierCounter inlierCounter;

        std::unique_ptr<CorrespondenceGraph> correspondenceGraph;

        std::string relativePoseFileG2o = "relativeRotations.txt";

    private:


        /**
         * @param destinationPoints, transformedPoints aligned pointclouds
         * @param cameraIntrinsics cameraParameters of not destination pose
         *
         * @returns vector i-th element represents OX and OY reprojection errors for i-th point
         */
        static std::vector<std::pair<double, double>> getReprojectionErrorsXY(const Eigen::Matrix4Xd &destinationPoints,
                                                                              const Eigen::Matrix4Xd &transformedPoints,
                                                                              const CameraRGBD &cameraIntrinsics);

        /**
         * @param vertexFrom is a vertex number relative transformation from is computed
         * @param vertexInList is a vertex number in vertexFrom's adjacency list (transformation "destination" vertex)
         * @param transformation is relative SE3 transformation between poses
         *
         * @returns information about matched keypoints
         */
        KeyPointMatches
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       const SE3 &transformation) const;

        /** Refine relative pose estimation with ICP-like dense clouds alignment
         * @param[in] vertexToBeTransformed pose which is transformed by SE3 transformation
         * @param[in] vertexDestination static destination pose
         * @param[in] keyPointMatches represents information about matched keyPoint
         *      first stored point is from transformed image and second from destination image
         * @param[in, out] initEstimationRelPos represents initial robust relative pose estimation,
         *      also stores refined estimation
         * @param[out] refinementSuccess true if refinement was successful
         *
         * @returns 0 if refinement was successful
         */
        int refineRelativePose(const VertexPose &vertexToBeTransformed,
                               const VertexPose &vertexDestination,
                               const KeyPointMatches &keyPointMatches,
                               SE3 &initEstimationRelPos,
                               bool &refinementSuccess) const;

        /** Get relative pose estimation between two poses with robust estimator
         * @param vertexFromDestOrigin[in] vertex index being transformed by SE3 transformation being estimated
         * @param vertexInListToBeTransformedCanBeComputed[in] vertex index in vertexFromDestOrigin's adjacency list
         * @param keyPointMatches[out] contains information about inlier matches between keypoints,
         *      each vector is size 2 and i={0,1}-th element contains information about point from image:
         *      {observing pose vertexIndex, keypoint index in pose's keypoint list, information about keypoint itself}
         * @param success[out] is true if estimation was successful
         * @param showMatchesOnImages[in] is true if keypoint matches should be visualized
         *
         * @returns SE3 optimal transformation
         */
        SE3
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin,
                                           int vertexInListToBeTransformedCanBeComputed,
                                           KeyPointMatches &keyPointMatches,
                                           bool &success,
                                           bool showMatchesOnImages = false) const;

        /** Compute all SE3 pairwise relative poses between N poses
         * @param[out] list of all inlier keypoint matches
         *      each vector is size 2 and i={0,1}-th element contains information about point from image:
         *      {observing pose vertexIndex, keypoint index in pose's keypoint list, information about keypoint itself}
         *
         * @returns N vectors where i-th vector contains all successfully estimated transformations from i-th pose
         */
        std::vector<std::vector<RelativeSE3>> findTransformationRtMatrices(
                KeyPointMatches &allInlierKeyPointMatches) const;

    public:

        bool getPrintInformationCout() const;

        void setPrintInformationCout(bool printProgressToCout);

        /**
         * @param datasetDescriber contains information about paths to RGB and D images, timestamps and camera intrinsics
         * @param cameraDefault camera intrinsics used by default for all cameras
         */
        RelativePosesComputationHandler(const DatasetCameraDescriber &datasetDescriber,
                                        const ParamsRANSAC &paramsRansac = ParamsRANSAC());

        /** Compute SE3 relative poses between all poses with LoRANSAC keypoint based procedure
         *      and ICP dense alignment refinement
         * @returns N vectors where i-th vector contains all successfully estimated transformations from i-th pose
         */
        std::vector<std::vector<RelativeSE3>> computeRelativePoses(
                const std::vector<int> &gpuDeviceIndices = {0}
                );

        /**
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components's vertcies indices
         */
        std::vector<std::vector<int>> bfsComputeConnectedComponents(
                std::vector<int> &componentNumberByPoseIndex) const;

        /** Compute pose graph connected components i.e. areas that can be reconstructed without ambiguity
         * @returns vector of further reconstruction handlers (absolute poses computation)
         */
        std::vector<std::unique_ptr<AbsolutePosesComputationHandler>> splitGraphToConnectedComponents() const;


        /**
         * @param outFile file name graph will be written to
         * @returns vector of connected components of graph
         */
        void bfsDrawToFile(const std::string &outFile);

        void setPathRelativePoseFile(const std::string &relativePoseFilePath);

        const std::string &getPathRelativePose() const;

        int getNumberOfVertices() const;

        const CorrespondenceGraph &getCorrespondenceGraph() const;

        void setNumberOfThreadsCPU(int numberOfThreadsCPU);

        void setDeviceCudaICP(int deviceCudaIcpToSet);

        void printTimeBenchmarkInfo() const;
    };
}

#endif
