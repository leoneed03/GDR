//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "directoryTraversing/DirectoryReader.h"
#include "poseGraph/CorrespondenceGraph.h"
#include "printer.h"
#include "relativePoseRefinement/ICP.h"
#include "visualization/2D/ImageDrawer.h"
#include "keyPointDetectionAndMatching/KeyPointsAndDescriptors.h"
#include "keyPointDetectionAndMatching/FeatureDetector.h"
#include "relativePoseEstimators/EstimatorRobustLoRANSAC.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <absolutePoseEstimation/translationAveraging/translationAveraging.h>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

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
            int numPoses = transformationRtMatrices.size();
            for (int i = 0; i < numPoses; ++i) {
                std::string s1 = "VERTEX_SE3:QUAT ";
                std::string s2 = std::to_string(i) + " 0.000000 0.000000 0.000000 0.0 0.0 0.0 1.0\n";
                file << s1 + s2;
            }
            for (int i = 0; i < transformationRtMatrices.size(); ++i) {
                for (int j = 0; j < transformationRtMatrices[i].size(); ++j) {
                    if (i >= transformationRtMatrices[i][j].getIndexTo()) {
                        continue;
                    }
                    std::string noise = "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000";

                    int indexTo = transformationRtMatrices[i][j].getIndexTo();
                    int indexFrom = i;
                    //order of vertices in the EDGE_SE3:QUAT representation is reversed (bigger_indexTo less_indexFrom)
                    // (gtsam format)
                    file << "EDGE_SE3:QUAT " << indexFrom << ' ' << indexTo << ' ';
                    auto translationVector = transformationRtMatrices[i][j].getRelativeTranslation();
                    file << ' ' << std::to_string(translationVector.col(0)[0]) << ' '
                         << std::to_string(translationVector.col(0)[1]) << ' '
                         << std::to_string(translationVector.col(0)[2]) << ' ';
                    const auto &qR = transformationRtMatrices[i][j].getRelativeRotation();

                    file << qR << ' ' << noise << '\n';
                }
            }
        }

        return 0;
    }


    CorrespondenceGraph::CorrespondenceGraph(const std::string &newPathToImageDirectoryRGB,
                                             const std::string &newPathToImageDirectoryD,
                                             const CameraRGBD &cameraDefaultToSet,
                                             int numOfThreadsCpu) :
            cameraDefault(cameraDefaultToSet),
            pathToImageDirectoryRGB(newPathToImageDirectoryRGB),
            pathToImageDirectoryD(newPathToImageDirectoryD) {

        std::cout << "construct Graph" << std::endl;
        imagesRgb = DirectoryReader::readPathsToImagesFromDirectory(pathToImageDirectoryRGB);
        imagesD = DirectoryReader::readPathsToImagesFromDirectory(pathToImageDirectoryD);
        std::cout << "data have been read" << std::endl;

        std::sort(imagesRgb.begin(), imagesRgb.end());
        std::sort(imagesD.begin(), imagesD.end());
        assert(imagesRgb.size() == imagesD.size());

        int numberOfPosesRead = imagesRgb.size();
        transformationRtMatrices = std::vector<std::vector<RelativeSE3>>(imagesD.size());
        verticesOfCorrespondence.reserve(numberOfPosesRead);

    }

    void CorrespondenceGraph::printConnectionsRelative(std::ostream &os, int space) const {

        int counter = 0;
        int counterSquared = 0;
        os << "EDGES of the Correspondence Graph:" << std::endl;
        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            os << std::setw(space / 5) << i << ":";
            counter += transformationRtMatrices[i].size();
            counterSquared += transformationRtMatrices[i].size() * transformationRtMatrices[i].size();

            for (int j = 0; j < transformationRtMatrices[i].size(); ++j) {
                const RelativeSE3 &e = transformationRtMatrices[i][j];
                assert(i == e.getIndexFrom());
                os << std::setw(space / 2) << e.getIndexTo() << ",";
            }
            os << std::endl;
        }
        os << "average number of edges " << counter / transformationRtMatrices.size() << std::endl;

        os << "sq D " << sqrt(counterSquared * 1.0 / transformationRtMatrices.size() -
                              pow(counter * 1.0 / transformationRtMatrices.size(), 2)) << std::endl;

    }

    void CorrespondenceGraph::setRelativePoses(const std::vector<std::vector<RelativeSE3>> &pairwiseRelativePoses) {
        assert(!transformationRtMatrices.empty());
        assert(transformationRtMatrices.size() == pairwiseRelativePoses.size());

        for (auto &transformations: transformationRtMatrices) {
            transformations.clear();
            assert(transformations.empty());
        }

        assert(transformationRtMatrices.size() == pairwiseRelativePoses.size());

        for (int i = 0; i < pairwiseRelativePoses.size(); ++i) {
            const auto &transformationsValuesToSet = pairwiseRelativePoses[i];
            auto &transformationsWhereToSet = transformationRtMatrices[i];

            for (int j = 0; j < transformationsValuesToSet.size(); ++j) {
                transformationRtMatrices[i].emplace_back(transformationsValuesToSet[j]);
            }
            assert(transformationsWhereToSet.size() == transformationsValuesToSet.size());
        }

    }

    void CorrespondenceGraph::setInlierPointMatches(
            const std::vector<std::array<std::pair<std::pair<int, int>, KeyPointInfo>, 2>> &inlierPointMatches) {
        inlierCorrespondencesPoints = inlierPointMatches;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsRGB() const {
        return imagesRgb;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsD() const {
        return imagesD;
    }

    int CorrespondenceGraph::getNumberOfPoses() const {

        int numberOfPoses = verticesOfCorrespondence.size();

        assert(numberOfPoses == imagesRgb.size());
        assert(numberOfPoses == imagesD.size());
        assert(numberOfPoses == transformationRtMatrices.size());

        return numberOfPoses;
    }

    const CameraRGBD &CorrespondenceGraph::getCameraDefault() const {
        return cameraDefault;
    }

    void CorrespondenceGraph::addVertex(const VertexCG &vertex) {
        verticesOfCorrespondence.emplace_back(vertex);
    }

    const std::vector<VertexCG> &CorrespondenceGraph::getVertices() const {
        assert(!verticesOfCorrespondence.empty());

        return verticesOfCorrespondence;
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
        assert(indexInMatchListToBeTransformedCanBeComputed >= 0 && indexInMatchListToBeTransformedCanBeComputed < matchList.size());

        return matchList[indexInMatchListToBeTransformedCanBeComputed];
    }

    void CorrespondenceGraph::setVertexCamera(int vertexIndex, const CameraRGBD &camera) {
        assert(vertexIndex >= 0 && vertexIndex < verticesOfCorrespondence.size());
        verticesOfCorrespondence[vertexIndex].setCamera(camera);
    }

    const std::vector<RelativeSE3> &CorrespondenceGraph::getConnectionsFromVertex(int vertexNumber) const {
        assert(transformationRtMatrices.size() == getNumberOfPoses());
        assert(vertexNumber >= 0 && vertexNumber < getNumberOfPoses());

        return transformationRtMatrices[vertexNumber];
    }

    const VertexCG &CorrespondenceGraph::getVertex(int vertexNumber) const {
        return verticesOfCorrespondence[vertexNumber];
    }

    const std::vector<std::array<std::pair<std::pair<int, int>, KeyPointInfo>, 2>> &
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
