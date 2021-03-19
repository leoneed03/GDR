//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SMOOTHPOINTCLOUD_H
#define GDR_SMOOTHPOINTCLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "poseGraph/VertexCG.h"

namespace gdr {

    struct SmoothPointCloud {
        void registerPointCloudFromImage(const std::vector<VertexCG*> &posesToBeRegistered,
                                         double voxelSizeX = 0.01,
                                         double voxelSizeY = 0.01,
                                         double voxelSixeZ = 0.01);
    };
}

#endif
