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
#include "parametrization/RelativeSE3.h"
#include "parametrization/cameraRGBD.h"
#include "keyPointDetectionAndMatching/ISiftModule.h"
#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "Vectors3d.h"
#include "ThreadPool.h"
#include "ConnectedComponent.h"
#include "relativePoseEstimators/IEstimatorRelativePoseRobust.h"
#include "relativePoseRefinement/IRefinerRelativePose.h"

namespace gdr {

    class CorrespondenceGraph {

        CameraRGBD cameraDefault;
        std::vector<VertexCG> verticesOfCorrespondence;

        std::vector<std::vector<Match>> matches;
        std::vector<std::vector<RelativeSE3>> transformationRtMatrices;

        const std::string relativePosesFileG2o = "relativeRotations.txt";
        const std::string absoluteRotationsFileG2o = "absoluteRotations.txt";
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

        void decreaseDensity(int maxVertexDegree = 80);

        void setRelativePoses(const std::vector<std::vector<RelativeSE3>> &pairwiseRelativePoses);

        void setInlierPointMatches(
                const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &inlierPointMatches);

        void setPointMatchesRGB(const std::vector<std::vector<Match>> &pointMatchesRGB);

        void setVertexCamera(int vertexIndex, const CameraRGBD &camera);

        void addVertex(const VertexCG &vertex);

        //TODO: bool addEdge(const RelativeSE3&)

    public:

        const std::string &getPathRelativePoseFile() const;

        const std::string &getPathAbsoluteRotationsFile() const;

        const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &getInlierObservedPoints() const;

        const VertexCG &getVertex(int vertexNumber) const;

        const std::vector<RelativeSE3> &getConnectionsFromVertex(int vertexNumber) const;

        const std::vector<std::vector<Match>> &getKeyPointMatches() const;

        const std::vector<VertexCG> &getVertices() const;

        const CameraRGBD &getCameraDefault() const;

        int getNumberOfPoses() const;

        const std::vector<std::string> &getPathsRGB() const;

        const std::vector<std::string> &getPathsD() const;

        void printConnectionsRelative(std::ostream &os, int space = 10) const;

        int printRelativePosesFile(const std::string &outPutFileRelativePoses) const;
    };
}

#endif
