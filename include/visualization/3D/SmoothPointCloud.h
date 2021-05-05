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
                                                double voxelSizeX = 0.01,
                                                double voxelSizeY = 0.01,
                                                double voxelSixeZ = 0.01,
                                                const std::string &pathPlyToSave = "",
                                                const std::string &screenshotPath = "");
    };
}

#endif
