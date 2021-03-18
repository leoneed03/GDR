//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SIFTGPU_CG_H
#define GDR_SIFTGPU_CG_H

#include <queue>
#include <atomic>
#include <tbb/concurrent_unordered_map.h>

#include "sparsePointCloud/CloudProjector.h"
#include "VertexCG.h"
#include "parametrization/transformationRt.h"
#include "parametrization/cameraRGBD.h"
#include "keyPointDetectionAndMatching/ISiftModule.h"
#include "rotationAveraging.h"
#include "quaternions.h"
#include "errors.h"
#include "umeyama.h"
#include "Vectors3d.h"
#include "ThreadPool.h"
#include "parametrization/RelativePoseSE3.h"
#include "ConnectedComponent.h"
#include "relativePoseEstimators/IEstimatorRelativePoseRobust.h"
#include "relativePoseRefinement/IRefinerRelativePose.h"

namespace gdr {

    struct CorrespondenceGraph {

        std::unique_ptr<ThreadPool> threadPool;
//        PointMatcher pointMatcher;
//        CloudProjector cloudProjector;
        CameraRGBD cameraRgbd;
        std::unique_ptr<ISiftModule> siftModule;
        std::vector<VertexCG> verticesOfCorrespondence;
        std::unique_ptr<IEstimatorRelativePoseRobust> relativePoseEstimatorRobust;
        std::unique_ptr<IRefinerRelativePose> relativePoseRefiner;

        // for i-th element vector should contain {component number, index}
        // index is an integer value from 0 to component.size() - 1 incl.
//        std::vector<std::pair<int, int>> connectedComponentNumberAndInsideIndex;
        int maxVertexDegree = 80;
        int numIterations = 100;
        std::vector<std::vector<Match>> matches;
        std::vector<std::vector<transformationRtMatrix>> transformationRtMatrices;
        tbb::concurrent_vector<tbb::concurrent_vector<int>> pairsWhereGotBetterResults;
        tbb::concurrent_vector<tbb::concurrent_unordered_map<int, transformationRtMatrix>> transformationMatricesLoRansac;
        tbb::concurrent_vector<tbb::concurrent_unordered_map<int, transformationRtMatrix>> transformationMatricesICP;
        double neighbourhoodRadius = 0.05;
        // TODO: check parameters again -- we need approx. 15 [inlier!] matches
        int minNumberOfMatches = 15;
        const std::string redCode = "\033[0;31m";
        const std::string resetCode = "\033[0m";
        const std::string relativePose = "relativeRotations.txt";
        const std::string absolutePose = "absoluteRotations.txt";
        std::vector<std::string> imagesRgb;
        std::vector<std::string> imagesD;
        std::string pathToImageDirectoryRGB;
        std::string pathToImageDirectoryD;
        std::atomic_int totalMeausedRelativePoses = 0;
        std::atomic_int refinedPoses = 0;

//        const CloudProjector &getCloudProjector() const;

        tbb::concurrent_vector<tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondencesPoints;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       double inlierCoeff,
                                       const SE3 &transformation,
                                       bool useProjection,
                                       double maxProjectionErrorPixels);

        CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                            const std::string &pathToImageDirectoryD,
                            float fx, float cx,
                            float fy, float cy,
                            int numOfThreadsCpu = 4);

        int refineRelativePose(const VertexCG &vertexToBeTransformed, const VertexCG &vertexDestination,
                               SE3 &initEstimationRelPos,
                               bool &refinementSuccess);

        int findTransformationRtMatrices();

        void decreaseDensity();

        // each pair is poseNumber and point's local index
        // paired with its KeyPointInfo
        // pairs are grouped in one vector if representing same global point

//        void computePointClasses(
//                const tbb::concurrent_vector<tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &matchesBetweenPoints);
//
//        void computePointClasses();

        Eigen::Matrix4d
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin, int vertexInListToBeTransformedCanBeComputed,
                                           bool &success,
                                           bool useProjection = true,
                                           double inlierCoeff = 0.5,
                                           double maxProjectionErrorPixels = 2.0,
                                           bool showMatchesOnImages = false);

        void printConnectionsRelative(std::ostream &os, int space = 10);
//
//        int printAbsolutePoses(std::ostream &os, int space = 10);

//        std::vector<Eigen::Quaterniond> performRotationAveraging();

        int printRelativePosesFile(const std::string &outPutFileRelativePoses);

        std::vector<int> bfs(int currentVertex,
                             bool &isConnected,
                             std::vector<std::vector<int>> &connectivityComponents);

        // return vectors -- each vector represents one connected component
        std::vector<std::vector<int>> bfsDrawToFile(const std::string &outFile) const;

        /**
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components
         */

        std::vector<std::vector<int>> bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const;

        int computeRelativePoses();

        std::vector<Eigen::Matrix4d> getAbsolutePosesEigenMatrix4d() const;

//        std::vector<Eigen::Vector3d> optimizeAbsoluteTranslations(int indexFixedToZero = 0);

//        std::vector<Sophus::SE3d> performBundleAdjustmentUsingDepth(int indexFixedToZero = 0);

        std::vector<ConnectedComponentPoseGraph> splitGraphToConnectedComponents() const;
    };
}

#endif
