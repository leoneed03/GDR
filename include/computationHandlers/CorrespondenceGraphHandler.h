//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CORRESPONDENCEGRAPHHANDLER_H
#define GDR_CORRESPONDENCEGRAPHHANDLER_H

#include "poseGraph/CorrespondenceGraph.h"

namespace gdr {
    class CorrespondenceGraphHandler {

        int numberOfThreadsCPU = 1;

        std::unique_ptr<ISiftModule> siftModule;
        std::unique_ptr<IEstimatorRelativePoseRobust> relativePoseEstimatorRobust;
        std::unique_ptr<IRefinerRelativePose> relativePoseRefiner;
        std::unique_ptr<ThreadPool> threadPool;
        ParamsRANSAC paramsRansac;

        std::unique_ptr<CorrespondenceGraph> correspondenceGraph;

        std::string relativePoseFileG2o = "relativeRotations.txt";

    private:


        static std::vector<std::pair<double, double>> getReprojectionErrorsXY(const Eigen::Matrix4Xd &destinationPoints,
                                                                              const Eigen::Matrix4Xd &transformedPoints,
                                                                              const CameraRGBD &cameraIntrinsics);

        /**
         * @param vertexFrom is a vertex number relative transformation from is computed
         * @param vertexInList is a vertex number in vertexFrom's adjacency list (transformation "destination" vertex)
         * @param maxErrorL2 is max L2 error in meters for points to be counted as inliers
         * @param transformation is relative SE3 transformation between poses
         * @param useProjectionError is "true" if reprojection error should be used instead of L2 error
         * @param maxProjectionErrorPixels is max reprojection error value for point pair to be an inlier
         * @param p is parameter for Lp reprojection error metric
         * @returns list of vectors each vector size is 2: for keypoint on the first image and the second
         *      pair is {{observingPoseNumber, keyPointLocalIndexOnTheImage}, KeyPointInfo}
         */
        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       const SE3 &transformation,
                                       const ParamsRANSAC &paramsRansac) const;

        int refineRelativePose(const VertexCG &vertexToBeTransformed,
                               const VertexCG &vertexDestination,
                               SE3 &initEstimationRelPos,
                               bool &refinementSuccess) const;

        SE3
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin,
                                           int vertexInListToBeTransformedCanBeComputed,
                                           std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &keyPointMatches,
                                           bool &success,
                                           const ParamsRANSAC &paramsRansac,
                                           bool showMatchesOnImages = false) const;

        std::vector<std::vector<RelativeSE3>> findTransformationRtMatrices(
                std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &vertexNumberFrom) const;

    public:


        std::vector<std::vector<RelativeSE3>> computeRelativePoses();

        void setPathRelativePoseFile(const std::string &relativePoseFilePath);

        const std::string &getPathRelativePose() const;

        int getNumberOfVertices() const;

    public:
        CorrespondenceGraphHandler(const std::string &pathToImageDirectoryRGB,
                                   const std::string &pathToImageDirectoryD,
                                   const CameraRGBD &cameraDefault = CameraRGBD());

        const CorrespondenceGraph &getCorrespondenceGraph() const;

        void setNumberOfThreadsCPU(int numberOfThreadsCPU);
    };
}

#endif
