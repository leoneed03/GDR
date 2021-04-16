//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SMOOTHPOINTCLOUD_H
#define GDR_SMOOTHPOINTCLOUD_H

#include "poseGraph/VertexCG.h"

namespace gdr {

    struct PointXYZRGBfloatUchar {
        float x, y, z;
        unsigned char R, G, B;

        PointXYZRGBfloatUchar(float X1, float Y1, float Z1,
                              unsigned char R1, unsigned char G1, unsigned char B1) :
                x(X1), y(Y1), z(Z1),
                R(R1), G(G1), B(B1) {};
    };

    struct SmoothPointCloud {
        //TODO: store pointCloud as tree-like structure
        static int registerPointCloudFromImages(const std::vector<VertexCG> &posesToBeRegistered,
                                                bool showVisualization = false,
                                                float voxelSizeX = 0.01,
                                                float voxelSizeY = 0.01,
                                                float voxelSixeZ = 0.01,
                                                const std::string &pathPlyToSave = "");

        static std::vector<PointXYZRGBfloatUchar> getPointCloudXYZRGBFromPose(const VertexCG &poseToBeRegistered);
    };
}

#endif
