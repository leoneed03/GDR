//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CORRESPONDENCE_GRAPH_H
#define GDR_CORRESPONDENCE_GRAPH_H

#include "poseGraph/PoseGraph.h"

#include "sparsePointCloud/CloudProjector.h"
#include "VertexCG.h"
#include "parametrization/RelativeSE3.h"
#include "cameraModel/CameraRGBD.h"
#include "keyPointDetectionAndMatching/IFeatureDetectorMatcher.h"
#include "absolutePoseEstimation/rotationAveraging/RotationAverager.h"
#include "parametrization/Vectors3d.h"
#include "computationHandlers/ThreadPool.h"
#include "ConnectedComponent.h"

#include "relativePoseEstimators/IEstimatorRelativePoseRobust.h"
#include "relativePoseRefinement/IRefinerRelativePose.h"

namespace gdr {

    class CorrespondenceGraph {

        CameraRGBD cameraDefault;

        PoseGraph poseGraph;

        std::vector<std::vector<Match>> matches;

        const std::string relativePosesFileG2o = "relativeRotations.txt";
        const std::string absoluteRotationsFileG2o = "absoluteRotations.txt";
        std::vector<std::string> imagesRgb;
        std::vector<std::string> imagesD;

        KeyPointMatches inlierCorrespondencesPoints;

    public:

        CorrespondenceGraph(const std::vector<std::string> &associatedImagesRGB,
                            const std::vector<std::string> &associatedImagesD,
                            const CameraRGBD &cameraDefault);

        const Match &getMatch(int indexFromDestDestination, int indexInMatchListToBeTransformedCanBeComputed) const;

        void decreaseDensity(int maxVertexDegree = 80);

        void setRelativePoses(const std::vector<std::vector<RelativeSE3>> &pairwiseRelativePoses);

        void setInlierPointMatches(
                const KeyPointMatches &inlierPointMatches);

        void setPointMatchesRGB(const std::vector<std::vector<Match>> &pointMatchesRGB);

        void setVertexCamera(int vertexIndex, const CameraRGBD &camera);

        void addVertex(const VertexCG &vertex);

        const std::string &getPathRelativePoseFile() const;

        const std::string &getPathAbsoluteRotationsFile() const;

        const KeyPointMatches &getInlierObservedPoints() const;

        const VertexCG &getVertex(int vertexNumber) const;

        const std::vector<RelativeSE3> &getConnectionsFromVertex(int vertexNumber) const;

        const std::vector<std::vector<Match>> &getKeyPointMatches() const;

        const std::vector<VertexCG> &getVertices() const;

        const CameraRGBD &getCameraDefault() const;

        int getNumberOfPoses() const;

        const std::vector<std::string> &getPathsRGB() const;

        const std::vector<std::string> &getPathsD() const;

        void printConnectionsRelative(std::ostream &os, int space = 10) const;
    };
}

#endif
