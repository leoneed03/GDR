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
#include "RelativeSE3.h"
#include "parametrization/cameraRGBD.h"
#include "keyPointDetectionAndMatching/ISiftModule.h"
#include "absolutePoseEstimation/rotationAveraging/rotationAveraging.h"
#include "Vectors3d.h"
#include "ThreadPool.h"
#include "parametrization/RelativePoseSE3.h"
#include "ConnectedComponent.h"
#include "relativePoseEstimators/IEstimatorRelativePoseRobust.h"
#include "relativePoseRefinement/IRefinerRelativePose.h"

namespace gdr {

    struct CorrespondenceGraph {

        std::unique_ptr<ThreadPool> threadPool;
        CameraRGBD cameraRgbd;
        std::unique_ptr<ISiftModule> siftModule;
        std::vector<VertexCG> verticesOfCorrespondence;
        std::unique_ptr<IEstimatorRelativePoseRobust> relativePoseEstimatorRobust;
        std::unique_ptr<IRefinerRelativePose> relativePoseRefiner;
        ParamsRANSAC paramsRansac;

        int maxVertexDegree = 80;
        std::vector<std::vector<Match>> matches;
        std::vector<std::vector<RelativeSE3>> transformationRtMatrices;
        tbb::concurrent_vector<tbb::concurrent_vector<int>> pairsWhereGotBetterResults;
        tbb::concurrent_vector<tbb::concurrent_unordered_map<int, RelativeSE3>> transformationMatricesLoRansac;
        tbb::concurrent_vector<tbb::concurrent_unordered_map<int, RelativeSE3>> transformationMatricesICP;

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

        tbb::concurrent_vector<tbb::concurrent_vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondencesPoints;


    public:

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>
        findInlierPointCorrespondences(int vertexFrom,
                                       int vertexInList,
                                       const SE3 &transformation,
                                       const ParamsRANSAC &paramsRansac);

        CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                            const std::string &pathToImageDirectoryD,
                            float fx, float cx,
                            float fy, float cy,
                            int numOfThreadsCpu = 4);

        int refineRelativePose(const VertexCG &vertexToBeTransformed,
                               const VertexCG &vertexDestination,
                               SE3 &initEstimationRelPos,
                               bool &refinementSuccess);

        int findTransformationRtMatrices();

        void decreaseDensity();

        Eigen::Matrix4d
        getTransformationRtMatrixTwoImages(int vertexFromDestOrigin, int vertexInListToBeTransformedCanBeComputed,
                                           bool &success,
                                           const ParamsRANSAC &paramsRansac,
                                           bool showMatchesOnImages = false);

        void printConnectionsRelative(std::ostream &os, int space = 10);

        int printRelativePosesFile(const std::string &outPutFileRelativePoses);

        void bfsDrawToFile(const std::string &outFile) const;

        /**
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components
         */

        std::vector<std::vector<int>> bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const;

        int computeRelativePoses();

        std::vector<ConnectedComponentPoseGraph> splitGraphToConnectedComponents() const;
    };
}

#endif
