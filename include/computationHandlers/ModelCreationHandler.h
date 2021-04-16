//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_MODELCREATIONHANDLER_H
#define GDR_MODELCREATIONHANDLER_H

#include "poseGraph/PoseGraph.h"

#include "visualization/3D/SmoothPointCloud.h"

namespace gdr {

    class ModelCreationHandler {
        PoseGraph poseGraphToBeProcessed;

        float voxelSizeX = 0.01;
        float voxelSizeY = 0.01;
        float voxelSizeZ = 0.01;

    public:
        ModelCreationHandler(const PoseGraph &poseGraph);

        void setVoxelSize(float x, float y, float z);

        void visualize() const;

        int saveAsPly(const std::string &plyFilePath) const;
    };
}


#endif
