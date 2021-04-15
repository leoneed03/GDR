//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "poseGraph/CorrespondenceGraph.h"

#include "relativePoseRefinement/ICPCUDA.h"
#include "keyPointDetectionAndMatching/FeatureDetectorMatcherCreator.h"
#include "relativePoseEstimators/EstimatorRobustLoRANSAC.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <absolutePoseEstimation/translationAveraging/TranslationAverager.h>

#include "boost/graph/graphviz.hpp"

namespace gdr {

    void CorrespondenceGraph::decreaseDensity(int maxVertexDegree) {

        for (std::vector<Match> &correspondenceList: matches) {

            std::sort(correspondenceList.begin(), correspondenceList.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.getSize() > rhs.getSize();
            });

            if (correspondenceList.size() > maxVertexDegree) {
                std::vector<Match> newMatchList(correspondenceList.begin(),
                                                correspondenceList.begin() + maxVertexDegree);
                std::swap(correspondenceList, newMatchList);
            }
        }
    }

    int CorrespondenceGraph::printRelativePosesFile(const std::string &pathOutRelativePoseFile) const {

        std::ofstream file(pathOutRelativePoseFile);

        if (file.is_open()) {
            int numPoses = poseGraph.size();

            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }

            for (int i = 0; i < numPoses; ++i) {
                for (int j = 0; j < poseGraph.getNumberOfAdjacentVertices(i); ++j) {

                    int indexTo = poseGraph.getRelativePose(i, j).getIndexTo();

                    const auto &relativePose = poseGraph.getRelativePose(i, j);
                    if (i >= relativePose.getIndexTo()) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)

                    file << "EDGE_SE3:QUAT " << indexFrom << ' ' << indexTo << ' ';
                    auto translationVector = relativePose.getRelativeTranslation();
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &qR = relativePose.getRelativeRotation();

                    file << qR << ' ' << noise << '\n';
                }
            }
        }

        return 0;
    }


    CorrespondenceGraph::CorrespondenceGraph(const std::vector<std::string> &associatedImagesRGB,
                                             const std::vector<std::string> &associatedImagesD,
                                             const CameraRGBD &cameraDefaultToSet) :
            cameraDefault(cameraDefaultToSet) {

        assert(!associatedImagesRGB.empty());
        assert(associatedImagesD.size() == associatedImagesRGB.size());

        imagesRgb = associatedImagesRGB;
        imagesD = associatedImagesD;

        std::sort(imagesRgb.begin(), imagesRgb.end());
        std::sort(imagesD.begin(), imagesD.end());
        assert(imagesRgb.size() == imagesD.size());

    }

    void CorrespondenceGraph::printConnectionsRelative(std::ostream &os, int space) const {

        int counter = 0;
        int counterSquared = 0;
        os << "EDGES of the Correspondence Graph:" << std::endl;

        int sizePoseGraph = poseGraph.size();
        for (int i = 0; i < sizePoseGraph; ++i) {

            os << std::setw(space / 5) << i << ":";

            int adjacentNumber = poseGraph.getNumberOfAdjacentVertices(i);
            counter += adjacentNumber;
            counterSquared += adjacentNumber * adjacentNumber;

            for (int j = 0; j < poseGraph.getNumberOfAdjacentVertices(i); ++j) {
                const RelativeSE3 &e = poseGraph.getRelativePose(i, j);
                assert(i == e.getIndexFrom());
                os << std::setw(space / 2) << e.getIndexTo() << ",";
            }
            os << std::endl;
        }
        os << "average number of edges " << counter / sizePoseGraph << std::endl;

        os << "sq D " << sqrt(counterSquared * 1.0 / sizePoseGraph -
                              pow(counter * 1.0 / sizePoseGraph, 2)) << std::endl;

    }

    void CorrespondenceGraph::setRelativePoses(const std::vector<std::vector<RelativeSE3>> &pairwiseRelativePoses) {

        poseGraph.setRelativePoses(pairwiseRelativePoses);
    }

    void CorrespondenceGraph::setInlierPointMatches(
            const KeyPointMatches &inlierPointMatches) {
        inlierCorrespondencesPoints = inlierPointMatches;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsRGB() const {
        return imagesRgb;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsD() const {
        return imagesD;
    }

    int CorrespondenceGraph::getNumberOfPoses() const {

        int numberOfPoses = poseGraph.size();

        assert(numberOfPoses == imagesRgb.size());
        assert(numberOfPoses == imagesD.size());

        return numberOfPoses;
    }

    const CameraRGBD &CorrespondenceGraph::getCameraDefault() const {
        return cameraDefault;
    }

    void CorrespondenceGraph::addVertex(const VertexCG &vertex) {

        poseGraph.addPoseVertex(vertex);
    }

    const std::vector<VertexCG> &CorrespondenceGraph::getVertices() const {

        return poseGraph.getPoseVertices();
    }

    void CorrespondenceGraph::setPointMatchesRGB(const std::vector<std::vector<Match>> &pointMatchesRGB) {
        matches = pointMatchesRGB;
    }

    const std::vector<std::vector<Match>> &CorrespondenceGraph::getKeyPointMatches() const {
        return matches;
    }

    const Match &CorrespondenceGraph::getMatch(int indexFromDestination,
                                               int indexInMatchListToBeTransformedCanBeComputed) const {
        assert(indexFromDestination >= 0 && indexFromDestination < matches.size());
        const auto &matchList = matches[indexFromDestination];
        assert(indexInMatchListToBeTransformedCanBeComputed >= 0 &&
               indexInMatchListToBeTransformedCanBeComputed < matchList.size());

        return matchList[indexInMatchListToBeTransformedCanBeComputed];
    }

    void CorrespondenceGraph::setVertexCamera(int vertexIndex, const CameraRGBD &camera) {

        poseGraph.setCamera(vertexIndex, camera);
    }

    const std::vector<RelativeSE3> &CorrespondenceGraph::getConnectionsFromVertex(int vertexNumber) const {

        return poseGraph.getRelativePosesFrom(vertexNumber);
    }

    const VertexCG &CorrespondenceGraph::getVertex(int vertexNumber) const {
        return poseGraph.getPoseVertex(vertexNumber);
    }

    const KeyPointMatches &
    CorrespondenceGraph::getInlierObservedPoints() const {
        return inlierCorrespondencesPoints;
    }

    const std::string &CorrespondenceGraph::getPathRelativePoseFile() const {
        return relativePosesFileG2o;
    }

    const std::string &CorrespondenceGraph::getPathAbsoluteRotationsFile() const {
        return absoluteRotationsFileG2o;
    }
}
