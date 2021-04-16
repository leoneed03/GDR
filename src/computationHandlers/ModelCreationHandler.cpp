//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/ModelCreationHandler.h"

namespace gdr {

    ModelCreationHandler::ModelCreationHandler(const gdr::PoseGraph &poseGraph) :
            poseGraphToBeProcessed(poseGraph) {}

    void ModelCreationHandler::visualize() const {
        SmoothPointCloud::registerPointCloudFromImages(poseGraphToBeProcessed.getPoseVertices(),
                                                       true,
                                                       voxelSizeX,
                                                       voxelSizeY,
                                                       voxelSizeZ);
    }

    int ModelCreationHandler::saveAsPly(const std::string &plyFilePath) const {

        SmoothPointCloud::registerPointCloudFromImages(poseGraphToBeProcessed.getPoseVertices(),
                                                       false,
                                                       voxelSizeX,
                                                       voxelSizeY,
                                                       voxelSizeZ,
                                                       plyFilePath);

        return 0;
    }

    void ModelCreationHandler::setVoxelSize(float x, float y, float z) {
        voxelSizeX = x;
        voxelSizeY = y;
        voxelSizeZ = z;
    }
}