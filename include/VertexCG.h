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

#include <sophus/se3.hpp>

#include "keyFeatures.h"
#include "quaternions.h"
#include "cameraRGBD.h"


namespace gdr {

    struct VertexCG {

        CameraRGBD cameraRgbd;
        int index;
        Eigen::Matrix4d absoluteRotationTranslation;
        Sophus::SE3d absolutePose;
        std::vector<SiftGPU::SiftKeypoint> keypoints;
        std::vector<float> descriptors;
        std::vector<double> depths;
        std::string pathToRGBimage;
        std::string pathToDimage;
        int heightMirrorParameter = 480;
        int widthMirrorParameter = 640;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setRotation(const Eigen::Matrix3d &rotation);

        void setRotation(const Eigen::Quaterniond &rotationQuatd);

        void setTranslation(const Eigen::Vector3d &translation);

        void setRotationTranslation(const Eigen::Matrix4d &eigenRt);

        void setRotationTranslation(const Sophus::SE3d &sophusRt);


        Eigen::Quaterniond getRotationQuat() const;


        Eigen::Matrix4d getEigenMatrixAbsolutePose4d() const;

        VertexCG(int newIndex,
                 const CameraRGBD &newCameraRgbd,
                 const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                 const std::vector<float> &newDescriptors,
                 const std::vector<double> &newDepths,
                 const std::string &newPathRGB,
                 const std::string &newPathD);

        std::string getPathRGBImage() const;

        std::string getPathDImage() const;

        const CameraRGBD &getCamera() const;

        int getIndex() const;
    };
}

#endif
