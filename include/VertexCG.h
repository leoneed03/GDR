//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_VERTEXCG_H
#define GDR_VERTEXCG_H

#include <vector>
#include <iostream>
#include "SiftGPU.h"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU>
#include <Eigen/SVD>

#include "keyFeatures.h"
#include "quaternions.h"
#include "cameraRGBD.h"


namespace gdr {

    struct VertexCG {

        CameraRGBD cameraRgbd;
        int index;
        Eigen::Matrix4d absoluteRotationTranslation;
        std::vector<SiftGPU::SiftKeypoint> keypoints;
        std::vector<float> descriptors;
        std::vector<double> depths;
        std::string pathToRGBimage;
        std::string pathToDimage;
        int heightMirrorParameter = 480;
        int widthMirrorParameter = 640;

        void setRotation(const Eigen::Matrix3d &rotation);

        VertexCG(int newIndex,
                 const CameraRGBD& newCameraRgbd,
                 const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                 const std::vector<float> &newDescriptors,
                 const std::vector<double> &newDepths,
                 const std::string &newPathRGB,
                 const std::string &newPathD);
    };
}

#endif
