//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>

#include "poseGraph/graphAlgorithms/GraphTraverser.h"

namespace gdr {

    std::vector<std::unique_ptr<ConnectedComponentPoseGraph>>
    GraphTraverser::splitGraphToConnectedComponents(const CorrespondenceGraph &correspondenceGraph) {

        int numberOfVerticesCG = correspondenceGraph.getNumberOfPoses();

        std::vector<int> componentNumberByPose;
        std::vector<std::vector<int>> components = bfsComputeConnectedComponents(correspondenceGraph,
                                                                                 componentNumberByPose);

        for (int i = 0; i < components.size(); ++i) {

            for (const auto &pose: components[i]) {
                assert(componentNumberByPose[pose] == i);
            }
        }

        std::vector<std::unique_ptr<ConnectedComponentPoseGraph>> connectedComponents;

        // fill information about poses and change indices inside each components
        // so they are locally sequentially packed [0.. component.size() - 1]

        // components container
        std::vector<std::vector<VertexCG>> connectedComponentsVertices(components.size());

        // relative poses SE3 containers
        // each vector element represents one connected component connection (edges)
        // each vector of vectors is components.size() sized
        // and i-th vector containes edges from i-th pose (local index)
        std::vector<std::vector<std::vector<RelativeSE3>>> edgesOfComponentsByComponentsNumber(components.size());

        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            edgesOfComponentsByComponentsNumber[componentNumber].resize(components[componentNumber].size());
        }

        // contains pair {connected component number, "sequentially packed" index inside component}
        // for each pose
        std::vector<std::pair<int, int>> componentNumberAndLocalIndexByPoseGlobalIndex;

        for (int poseGlobalIndex = 0; poseGlobalIndex < numberOfVerticesCG; ++poseGlobalIndex) {

            VertexCG poseInsideConnectedComponent(correspondenceGraph.getVertex(poseGlobalIndex));

            int componentNumber = componentNumberByPose[poseGlobalIndex];
            int localIndexInsideComponent = connectedComponentsVertices[componentNumber].size();
            componentNumberAndLocalIndexByPoseGlobalIndex.emplace_back(
                    std::make_pair(componentNumber, localIndexInsideComponent));
            poseInsideConnectedComponent.setIndex(localIndexInsideComponent);

            connectedComponentsVertices[componentNumber].emplace_back(poseInsideConnectedComponent);
        }

        assert(componentNumberAndLocalIndexByPoseGlobalIndex.size() == numberOfVerticesCG);

        for (int globalPoseIndex = 0; globalPoseIndex < numberOfVerticesCG; ++globalPoseIndex) {
            const auto &infoAboutpose = componentNumberAndLocalIndexByPoseGlobalIndex[globalPoseIndex];
        }

        // recompute transformation matrices using new local indices
        for (int indexFrom = 0; indexFrom < numberOfVerticesCG; ++indexFrom) {
            for (const auto &transformation: correspondenceGraph.getConnectionsFromVertex(indexFrom)) {
                assert(transformation.getIndexFrom() == indexFrom);

                int indexTo = transformation.getIndexTo();
                int componentNumberIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].first;
                int localIndexFrom = componentNumberAndLocalIndexByPoseGlobalIndex[indexFrom].second;

                int componentNumberIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].first;
                int localIndexTo = componentNumberAndLocalIndexByPoseGlobalIndex[indexTo].second;

                assert(componentNumberIndexFrom == componentNumberIndexTo);

                const Sophus::SE3d &relativePoseSE3 = transformation.getRelativePoseSE3();
                RelativeSE3 localRelativePoseSE3(localIndexFrom,
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

        for (const auto &pairOfMatchedKeyPoints: correspondenceGraph.getInlierObservedPoints().getKeyPointMatchesVector()) {
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

        int addedVertices = 0;
        for (int componentNumber = 0; componentNumber < components.size(); ++componentNumber) {
            addedVertices += connectedComponentsVertices[componentNumber].size();
            int numberOfComponents = components.size();
            assert(numberOfComponents == connectedComponentsVertices.size());
            assert(numberOfComponents == edgesOfComponentsByComponentsNumber.size());
            assert(numberOfComponents == inlierCorrespondencesPointsInsideComponentByComponentNumber.size());
            std::string namePrefix =
                    "comp_" + std::to_string(numberWhenSortedBySizeByGlobalIndex[componentNumber]) + "_";

            connectedComponents.emplace_back(std::make_unique<
                    ConnectedComponentPoseGraph>(connectedComponentsVertices[componentNumber],
                                                 edgesOfComponentsByComponentsNumber[componentNumber],
                                                 inlierCorrespondencesPointsInsideComponentByComponentNumber[componentNumber],
                                                 namePrefix + correspondenceGraph.getPathRelativePoseFile(),
                                                 namePrefix + correspondenceGraph.getPathAbsoluteRotationsFile(),
                                                 componentNumber));
        }
        assert(addedVertices == correspondenceGraph.getNumberOfPoses());

        std::stable_sort(connectedComponents.begin(), connectedComponents.end(),
                         [](const auto &lhs, const auto &rhs) {
                             return lhs->getNumberOfPoses() > rhs->getNumberOfPoses();
                         });

        return connectedComponents;
    }

    std::vector<std::vector<int>>
    GraphTraverser::bfsComputeConnectedComponents(const CorrespondenceGraph &correspondenceGraph,
                                                  std::vector<int> &componentNumberByPoseIndex) {

        class V {
        };

        class C {
        };

        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;
        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (int poseNumber = 0; poseNumber < correspondenceGraph.getNumberOfPoses(); ++poseNumber) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < correspondenceGraph.getNumberOfPoses(); ++i) {
            for (const auto &edge: correspondenceGraph.getConnectionsFromVertex(i)) {
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

    void GraphTraverser::bfsDrawToFile(const CorrespondenceGraph &correspondenceGraph,
                                       const std::string &outFile) {

        class V {
        };
        class C {
        };
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> PoseGraphForBfs;
        typedef boost::graph_traits<PoseGraphForBfs>::vertex_descriptor VertexDescriptorForBfs;

        PoseGraphForBfs poseGraphForBfs;
        std::vector<VertexDescriptorForBfs> verticesBoost;

        for (const auto &pose: correspondenceGraph.getVertices()) {
            verticesBoost.push_back(boost::add_vertex(poseGraphForBfs));
        }

        for (int i = 0; i < correspondenceGraph.getNumberOfPoses(); ++i) {
            for (const auto &edge: correspondenceGraph.getConnectionsFromVertex(i)) {
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
}
