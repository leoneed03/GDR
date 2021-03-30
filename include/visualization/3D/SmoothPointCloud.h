//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SMOOTHPOINTCLOUD_H
#define GDR_SMOOTHPOINTCLOUD_H

#include "poseGraph/VertexCG.h"

namespace gdr {

    struct SmoothPointCloud {
        //TODO: export pointCloud as tree-like structure
        void registerPointCloudFromImage(const std::vector<VertexCG> &posesToBeRegistered,
                                         double voxelSizeX = 0.01,
                                         double voxelSizeY = 0.01,
                                         double voxelSixeZ = 0.01);
    };
}

#endif
