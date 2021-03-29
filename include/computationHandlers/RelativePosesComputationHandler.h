//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSESCOMPUTATIONHANDLER_H
#define GDR_RELATIVEPOSESCOMPUTATIONHANDLER_H

#include "computationHandlers/AbsolutePosesComputationHandler.h"

#include "poseGraph/graphAlgorithms/GraphTraverser.h"
#include "poseGraph/CorrespondenceGraph.h"

namespace gdr {

    class RelativePosesComputationHandler {

        bool printInformationConsole = false;
        int numberOfThreadsCPU = 1;

        std::unique_ptr<ISiftModule> siftModule;
        std::unique_ptr<IEstimatorRelativePoseRobust> relativePoseEstimatorRobust;
        std::unique_ptr<IRefinerRelativePose> relativePoseRefiner;
        // unused right now
        std::unique_ptr<ThreadPool> threadPool;
        CameraRGBD cameraDefault;
        ParamsRANSAC paramsRansac;

        std::unique_ptr<CorrespondenceGraph> correspondenceGraph;

        std::string relativePoseFileG2o = "relativeRotations.txt";

    private:


        /**
         * @param destinationPoints, transformedPoints aligned pointclouds
         * @param cameraIntrinsics cameraParameters of not destination pose
         * @returns vector i-th element represents OX and OY reprojection errors for i-th point
         */
        static std::vector<std::pair<double, double>> getReprojectionErrorsXY(const Eigen::Matrix4Xd &destinationPoints,
                                                                              const Eigen::Matrix4Xd &transformedPoints,
                                                                              const CameraRGBD &cameraIntrinsics);

        /**
         * @param vertexFrom is a vertex number relative transformation from is computed
         * @param vertexInList is a vertex number in vertexFrom's adjacency list (transformation "destination" vertex)
         * @param transformation is relative SE3 transformation between poses
         * @returns list of vectors each vector size is 2: for keypoint on the first image and the second
         *      pair is {{observingPoseNumber, keyPointLocalIndexOnTheImage}, KeyPointInfo}
         */
        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       const SE3 &transformation) const;

        /** Refine relative pose estimation with ICP-like dense clouds alignment
         * @param[in] vertexToBeTransformed pose which is transformed by SE3 transformation
         * @param[in] vertexDestination static pose
         * @param[in, out] initEstimationRelPos represents initial robust relative pose estimation,
         *      also stores refined estimation
         * @param[out] refinementSuccess true if refinement was successful
         */
        int refineRelativePose(const VertexCG &vertexToBeTransformed,
                               const VertexCG &vertexDestination,
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
         */
        SE3
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin,
                                           int vertexInListToBeTransformedCanBeComputed,
                                           std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatches,
                                           bool &success,
                                           bool showMatchesOnImages = false) const;

        /** Compute all SE3 pairwise relative poses between N poses
         * @param[out] list of all inlier keypoint matches
         *      each vector is size 2 and i={0,1}-th element contains information about point from image:
         *      {observing pose vertexIndex, keypoint index in pose's keypoint list, information about keypoint itself}
         * @returns N vectors where i-th vector contains all successfully estimated transformations from i-th pose
         */
        std::vector<std::vector<RelativeSE3>> findTransformationRtMatrices(
                std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &allInlierKeyPointMatches) const;

    public:
        bool getPrintInformationCout() const;

        void setPrintInformationCout(bool printProgressToCout);
        /**
         * @param pathToImageDirectoryRGB path to directory with N colour images
         * @param pathToImageDirectoryD path to directory with N depth images
         * @param cameraDefault camera intrinsics used by default for all cameras
         */
        RelativePosesComputationHandler(const std::string &pathToImageDirectoryRGB,
                                        const std::string &pathToImageDirectoryD,
                                        const ParamsRANSAC &paramsRansac = ParamsRANSAC(),
                                        const CameraRGBD &cameraDefault = CameraRGBD());

        /** Compute SE3 relative poses between all poses with LoRANSAC keypoint based procedure
         *      and ICP dense alignment refinement
         * @returns N vectors where i-th vector contains all successfully estimated transformations from i-th pose
         */
        std::vector<std::vector<RelativeSE3>> computeRelativePoses();

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
    };
}

#endif
