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
#include <tbb/parallel_for.h>
#include <absolutePoseEstimation/translationAveraging/translationAveraging.h>
#include <mutex>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <boost/graph/connected_components.hpp>

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

                    file << std::to_string(qR.x()) << ' ' << std::to_string(qR.y()) << ' ' <<
                         std::to_string(qR.z()) << ' '
                         << std::to_string(qR.w()) << noise << '\n';
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

        setNumberOfPoses(numberOfPosesRead);

        assert(getNumberOfPoses() == numberOfPosesRead);
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


    std::vector<std::vector<int>>
    CorrespondenceGraph::bfsComputeConnectedComponents(std::vector<int> &componentNumberByPoseIndex) const {

        class V {
        };

        class C {
        };

        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;
        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (const auto &pose: verticesOfCorrespondence) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            for (const auto &edge: transformationRtMatrices[i]) {
                assert(i == edge.getIndexFrom());

                if (edge.getIndexFrom() > edge.getIndexTo()) {
                    continue;
                }
                boost::add_edge(verticesBoost[edge.getIndexFrom()], verticesBoost[edge.getIndexTo()], poseGraphForBfs);
            }
        }

        std::vector<int> component(boost::num_vertices(poseGraphForBfs));
        size_t num_components = boost::connected_components(poseGraphForBfs, component.data());

        std::vector<std::vector<int>> components(num_components);

        for (int i = 0; i < component.size(); ++i) {
            components[component[i]].push_back(i);
        }

        componentNumberByPoseIndex = component;
        assert(!componentNumberByPoseIndex.empty());

        for (int componentNum = 0; componentNum < components.size(); ++componentNum) {
            for (const auto &pose: components[componentNum]) {
                assert(componentNumberByPoseIndex[pose] == componentNum);
            }
        }
        return components;
    }

    void CorrespondenceGraph::bfsDrawToFile(const std::string &outFile) const {

        class V {
        };
        class C {
        };
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;

        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (const auto &pose: verticesOfCorrespondence) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < transformationRtMatrices.size(); ++i) {
            for (const auto &edge: transformationRtMatrices[i]) {
                assert(i == edge.getIndexFrom());

                if (edge.getIndexFrom() > edge.getIndexTo()) {
                    continue;
                }
                boost::add_edge(verticesBoost[edge.getIndexFrom()], verticesBoost[edge.getIndexTo()], poseGraphForBfs);
            }
        }

        if (!outFile.empty()) {
            std::ofstream outf(outFile);
            boost::write_graphviz(outf, poseGraphForBfs);
        }
    }

    std::vector<ConnectedComponentPoseGraph> CorrespondenceGraph::splitGraphToConnectedComponents() const {

        std::vector<int> componentNumberByPose;
        std::vector<std::vector<int>> components = bfsComputeConnectedComponents(componentNumberByPose);

        for (int i = 0; i < components.size(); ++i) {
            std::cout << " component " << std::setw(4) << i << "-th size: " << components[i].size() << " elements: ";

            for (const auto &pose: components[i]) {
                std::cout << pose << ' ';
                assert(componentNumberByPose[pose] == i);
            }
            std::cout << std::endl;
        }
        std::vector<ConnectedComponentPoseGraph> connectedComponents;

        // fill information about poses and change indices inside each components
        // so they are locally sequentially packed [0.. component.size() - 1]

        // components container
        std::vector<std::vector<VertexCG>> connectedComponentsVertices(components.size());

        // relative poses SE3 containers
        // each vector element represents one connected component connections (edges)
        // each vector of vectors is components.size() sized
        // and i-th vector containes edges from i-th pose (local index)
        std::vector<std::vector<std::vector<RelativePoseSE3>>> edgesOfComponentsByComponentsNumber(components.size());

        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            edgesOfComponentsByComponentsNumber[componentNumber].resize(components[componentNumber].size());
        }
        // contains pair {connected component number, "sequentially packed" index inside component}
        // for each pose
        std::vector<std::pair<int, int>> componentNumberAndLocalIndexByPoseGlobalIndex;

        for (int poseGlobalIndex = 0; poseGlobalIndex < verticesOfCorrespondence.size(); ++poseGlobalIndex) {

            VertexCG poseInsideConnectedComponent(verticesOfCorrespondence[poseGlobalIndex]);
            int componentNumber = componentNumberByPose[poseGlobalIndex];
            int localIndexInsideComponent = connectedComponentsVertices[componentNumber].size();
            componentNumberAndLocalIndexByPoseGlobalIndex.emplace_back(
                    std::make_pair(componentNumber, localIndexInsideComponent));
            poseInsideConnectedComponent.setIndex(localIndexInsideComponent);

            connectedComponentsVertices[componentNumber].emplace_back(poseInsideConnectedComponent);
        }

        assert(componentNumberAndLocalIndexByPoseGlobalIndex.size() == verticesOfCorrespondence.size());

        for (int globalPoseIndex = 0; globalPoseIndex < verticesOfCorrespondence.size(); ++globalPoseIndex) {
            const auto &infoAboutpose = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndex];
            std::cout << "  INFO about #global pose " << globalPoseIndex
                      << " in #component " << infoAboutpose.first
                      << " index inside component is: " << infoAboutpose.second << std::endl;
        }
        assert(transformationRtMatrices.size() == verticesOfCorrespondence.size());

        // recompute transformation matrices using new local indices
        for (int indexFrom = 0; indexFrom < transformationRtMatrices.size(); ++indexFrom) {
            for (const auto &transformation: transformationRtMatrices[indexFrom]) {
                assert(transformation.getIndexFrom() == indexFrom);

                int indexTo = transformation.getIndexTo();
                int componentNumberIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].first;
                int localIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].second;

                int componentNumberIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].first;
                int localIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].second;

                assert(componentNumberIndexFrom == componentNumberIndexTo);

                const Sophus::SE3d &relativePoseSE3 = transformation.getRelativePoseSE3();
                RelativePoseSE3 localRelativePoseSE3(localIndexFrom,
                                                     localIndexTo,
                                                     transformation.getRelativePose());
                assert(componentNumberByPose[indexFrom] == componentNumberByPose[indexTo]);
                assert(componentNumberByPose[indexFrom] == componentNumberIndexTo);

                assert(localIndexFrom < edgesOfComponentsByComponentsNumber[componentNumberIndexFrom].size());
                assert(localRelativePoseSE3.getIndexFrom() == localIndexFrom);
                assert(localRelativePoseSE3.getIndexTo() == localIndexTo);
                edgesOfComponentsByComponentsNumber[componentNumberIndexTo][localIndexFrom].emplace_back(
                        localRelativePoseSE3);

            }
        }

        //fill information about observed points with local pose indices
        std::vector<std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>>>
                inlierCorrespondencesPointsInsideComponentByComponentNumber(components.size());

        for (const auto &pairOfMatchedKeyPoints: inlierCorrespondencesPoints) {
            assert(pairOfMatchedKeyPoints.size() == 2);
            int globalPoseIndexFirst = pairOfMatchedKeyPoints[0].first.first;
            int globalPoseIndexSecond = pairOfMatchedKeyPoints[1].first.first;

            int localPoseIndexFirst = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexFirst].second;
            int localPoseIndexSecond = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexSecond].second;
            int componentNumber = componentNumberByPose[globalPoseIndexFirst];

            const auto &pairFirst = pairOfMatchedKeyPoints[0];
            const auto &pairSecond = pairOfMatchedKeyPoints[1];
            KeyPointInfo keyPointInfoFirst = pairFirst.second;
            keyPointInfoFirst.setObservingPoseNumber(localPoseIndexFirst);
            KeyPointInfo keyPointInfoSecond = pairSecond.second;
            keyPointInfoSecond.setObservingPoseNumber(localPoseIndexSecond);

            std::pair<std::pair<int, int>, KeyPointInfo> pointInfoLocalFirst = {
                    {localPoseIndexFirst, pairFirst.first.second}, keyPointInfoFirst};
            std::pair<std::pair<int, int>, KeyPointInfo> pointInfoLocalSecond = {
                    {localPoseIndexSecond, pairSecond.first.second}, keyPointInfoSecond};

            assert(componentNumber == componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndexSecond].first);

            inlierCorrespondencesPointsInsideComponentByComponentNumber[componentNumber].push_back(
                    {pointInfoLocalFirst, pointInfoLocalSecond});
        }

        std::vector<std::pair<int, int>> componentSizeAndComponentNumber;
        for (int i = 0; i < components.size(); ++i) {
            componentSizeAndComponentNumber.emplace_back(std::make_pair(components[i].size(), i));
        }

        std::stable_sort(componentSizeAndComponentNumber.begin(),
                         componentSizeAndComponentNumber.end(),
                         std::greater<>());
        std::unordered_map<int, int> numberWhenSortedBySizeByGlobalIndex;

        for (int i = 0; i < componentSizeAndComponentNumber.size(); ++i) {
            auto &sizeAndIndex = componentSizeAndComponentNumber[i];
            numberWhenSortedBySizeByGlobalIndex[sizeAndIndex.second] = i;
        }
        for (const auto &indexAndIndexWhenSorted: numberWhenSortedBySizeByGlobalIndex) {
            std::cout << " for pose #indexed " << indexAndIndexWhenSorted.first << " sorted pose is "
                      << indexAndIndexWhenSorted.second << std::endl;
        }
        int addedVertices = 0;
        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            addedVertices += connectedComponentsVertices[componentNumber].size();
            int numberOfComponents = components.size();
            assert(numberOfComponents == connectedComponentsVertices.size());
            assert(numberOfComponents == edgesOfComponentsByComponentsNumber.size());
            assert(numberOfComponents == inlierCorrespondencesPointsInsideComponentByComponentNumber.size());
            int indexWhenSorted = numberWhenSortedBySizeByGlobalIndex[componentNumber];
            std::string namePrefix =
                    "comp_" + std::to_string(numberWhenSortedBySizeByGlobalIndex[componentNumber]) + "_";

            connectedComponents.emplace_back(
                    ConnectedComponentPoseGraph(connectedComponentsVertices[componentNumber],
                                                edgesOfComponentsByComponentsNumber[componentNumber],
                                                cameraDefault,
                                                inlierCorrespondencesPointsInsideComponentByComponentNumber[componentNumber],
                                                namePrefix + relativePoseFileG2o,
                                                namePrefix + absolutePose,
                                                componentNumber));
        }
        assert(addedVertices == verticesOfCorrespondence.size());

        std::stable_sort(connectedComponents.begin(), connectedComponents.end(),
                         [](const auto &lhs, const auto &rhs) {
                             return lhs.getNumberOfPoses() > rhs.getNumberOfPoses();
                         });

        return connectedComponents;
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
            const std::vector<std::vector<std::pair<std::pair<int, int>, KeyPointInfo>>> &inlierPointMatches) {
        inlierCorrespondencesPoints = inlierPointMatches;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsRGB() const {
        return imagesRgb;
    }

    const std::vector<std::string> &CorrespondenceGraph::getPathsD() const {
        return imagesD;
    }

    void CorrespondenceGraph::setNumberOfPoses(int numberOfPosesToSet) {
        numberOfPoses = numberOfPosesToSet;
    }

    int CorrespondenceGraph::getNumberOfPoses() const {
        assert(numberOfPoses > 0 && "no images to process");
        assert(numberOfPoses == imagesRgb.size());
        assert(numberOfPoses == imagesD.size());

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
}
