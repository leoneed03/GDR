//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/ModelCreationHandler.h"

namespace gdr {

    ModelCreationHandler::ModelCreationHandler(const gdr::PoseGraph &poseGraph) :
            poseGraphToBeProcessed(poseGraph) {}

    void ModelCreationHandler::visualize() const {
        std::vector<Reconstructable> posesToVisualize;

        for (const auto &poseVertex: poseGraphToBeProcessed.getPoseVertices()) {
            Reconstructable poseToVisualize(poseVertex.getPathRGBImage(),
                                            poseVertex.getPathDImage(),
                                            poseVertex.getCamera());
            poseToVisualize.setAbsolutePose(poseVertex.getAbsolutePoseSE3());

            posesToVisualize.emplace_back(poseToVisualize);
        }

        assert(posesToVisualize.size() == poseGraphToBeProcessed.size());

        SmoothPointCloud::registerPointCloudFromImages(posesToVisualize,
                                                       true,
                                                       voxelSizeX,
                                                       voxelSizeY,
                                                       voxelSizeZ);
    }

    int ModelCreationHandler::saveAsPly(const std::string &plyFilePath) const {
        std::vector<Reconstructable> posesToVisualize;

        for (const auto &poseVertex: poseGraphToBeProcessed.getPoseVertices()) {
            Reconstructable poseToVisualize(poseVertex.getPathRGBImage(),
                                            poseVertex.getPathDImage(),
                                            poseVertex.getCamera());
            poseToVisualize.setAbsolutePose(poseVertex.getAbsolutePoseSE3());

            posesToVisualize.emplace_back(poseToVisualize);
        }

        assert(posesToVisualize.size() == poseGraphToBeProcessed.size());
        SmoothPointCloud::registerPointCloudFromImages(posesToVisualize,
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