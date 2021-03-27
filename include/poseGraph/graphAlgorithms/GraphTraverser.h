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

        /**
         * @param[out] componentNumberByPoseIndex -- vector containing each pose's component number
         * @returns vector of vectors -- connected components
         */

        static std::vector<std::vector<int>> bfsComputeConnectedComponents(
                const CorrespondenceGraph &correspondenceGraph,
                std::vector<int> &componentNumberByPoseIndex);

        static
        std::vector<ConnectedComponentPoseGraph>
        splitGraphToConnectedComponents(const CorrespondenceGraph &correspondenceGraph);
    };
}

#endif
