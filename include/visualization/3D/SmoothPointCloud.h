//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SMOOTHPOINTCLOUD_H
#define GDR_SMOOTHPOINTCLOUD_H

#include "parametrization/Reconstructable.h"

namespace gdr {


    struct SmoothPointCloud {
        //TODO: store pointCloud as tree-like structure
        static int registerPointCloudFromImages(const std::vector<Reconstructable> &posesToBeRegistered,
                                                bool showVisualization = false,
                                                float voxelSizeX = 0.01,
                                                float voxelSizeY = 0.01,
                                                float voxelSixeZ = 0.01,
                                                const std::string &pathPlyToSave = "",
                                                const std::string &screenshotPath = "");
    };
}

#endif
