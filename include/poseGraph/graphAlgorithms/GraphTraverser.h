//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_GRAPHTRAVERSER_H
#define GDR_GRAPHTRAVERSER_H

#include <vector>

#include "poseGraph/CorrespondenceGraph.h"
#include "poseGraph/ConnectedComponent.h"

namespace gdr {
    class GraphTraverser {
    public:
        // TODO: depend on same interface for ConnectedComponent and CorrespondenceGraph:
        // getNumberOfPoses()
        // getConnectionsFromVertex(indexFrom)
        // getVertex(poseGlobalIndex)

        /**
         * @param[in] correspondenceGraph is pose graph itself to be traversed
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components
         */
        static std::vector<std::vector<int>> bfsComputeConnectedComponents(
                const CorrespondenceGraph &correspondenceGraph,
                std::vector<int> &componentNumberByPoseIndex);

        /**
         * @param correspondenceGraph is pose graph to be split to connected components
         * @returns vector of connected components of graph
         */
        static
        std::vector<std::unique_ptr<ConnectedComponentPoseGraph>>
        splitGraphToConnectedComponents(const CorrespondenceGraph &correspondenceGraph);

        /**
         * @param correspondenceGraph is pose graph to be shown
         * @param outFile file name graph will be written to
         * @returns vector of connected components of graph
         */
        static void bfsDrawToFile(const CorrespondenceGraph &correspondenceGraph,
                                  const std::string &outFile);
    };
}

#endif
