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

    class CorrespondenceGraph {

        int numberOfPoses = 0;
        CameraRGBD cameraDefault;
        std::vector<VertexCG> verticesOfCorrespondence;

        std::vector<std::vector<Match>> matches;
        std::vector<std::vector<RelativeSE3>> transformationRtMatrices;

        const std::string relativePoseFileG2o = "relativeRotations.txt";
        const std::string absolutePose = "absoluteRotations.txt";
        std::vector<std::string> imagesRgb;
        std::vector<std::string> imagesD;
        std::string pathToImageDirectoryRGB;
        std::string pathToImageDirectoryD;

        std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> inlierCorrespondencesPoints;

    public:

        CorrespondenceGraph(const std::string &pathToImageDirectoryRGB,
                            const std::string &pathToImageDirectoryD,
                            const CameraRGBD &cameraDefault,
                            int numOfThreadsCpu = 4);

        const Match &getMatch(int indexFromDestDestination, int indexInMatchListToBeTransformedCanBeComputed) const;

        std::vector<ConnectedComponentPoseGraph> splitGraphToConnectedComponents() const;

        void decreaseDensity(int maxVertexDegree = 80);

        void setRelativePoses(const std::vector<std::vector<RelativeSE3>> &pairwiseRelativePoses);

        void setInlierPointMatches(
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &inlierPointMatches);

        void setNumberOfPoses(int numberOfPoses);

        void setPointMatchesRGB(const std::vector<std::vector<Match>> &pointMatchesRGB);

        void setVertexCamera(int vertexIndex, const CameraRGBD &camera);

        void addVertex(const VertexCG &vertex);

        //TODO: bool addEdge(const RelativeSE3&)

    public:


        const std::vector<std::vector<Match>> &getKeyPointMatches() const;

        const std::vector<VertexCG> &getVertices() const;

        const CameraRGBD &getCameraDefault() const;

        int getNumberOfPoses() const;

        const std::vector<std::string> &getPathsRGB() const;

        const std::vector<std::string> &getPathsD() const;

        void printConnectionsRelative(std::ostream &os, int space = 10) const;

        int printRelativePosesFile(const std::string &outPutFileRelativePoses) const;

        void bfsDrawToFile(const std::string &outFile) const;

        /**
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components
         */

        std::vector<std::vector<int>> bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const;
    };
}

#endif
