//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_VERTEXCG_H
#define GDR_VERTEXCG_H

#include <vector>
#include <iostream>
//#include "SiftGPU.h"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU>
#include <Eigen/SVD>

#include <sophus/se3.hpp>

#include "KeyPointInfo.h"
#include "keyFeatures.h"
#include "quaternions.h"
#include "parametrization/SE3.h"
#include "parametrization/cameraRGBD.h"
#include "KeyPointsDepthDescriptor.h"


namespace gdr {

    struct VertexCG {

        CameraRGBD cameraRgbd;
        int index;
        int initialIndex;
//        Eigen::Matrix4d absoluteRotationTranslation;
        SE3 absolutePose;
        std::vector<KeyPoint2D> keypoints;
        std::vector<float> descriptors;
        std::vector<double> depths;
        std::string pathToRGBimage;
        std::string pathToDimage;
//        int heightMirrorParameter = 480;
//        int widthMirrorParameter = 640;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setIndex(int newIndex);

        void setRotation(const Sophus::SO3d &orientation);

        void setRotation(const Eigen::Matrix3d &orientation);

        void setRotation(const Eigen::Quaterniond &orientationQuatd);

        void setTranslation(const Eigen::Vector3d &translation);

        void setRotationTranslation(const Eigen::Matrix4d &eigenRt);

        void setRotationTranslation(const Sophus::SE3d &sophusRt);

        void setAbsolutePoseSE3(const SE3 &absolutePose);

        const std::vector<KeyPoint2D>& getKeyPoints2D() const;

        Eigen::Quaterniond getRotationQuat() const;

        Eigen::Matrix4d getEigenMatrixAbsolutePose4d() const;

        Sophus::SE3d getAbsolutePoseSophus() const;

        const SE3& getAbsolutePoseSE3() const;

        VertexCG(int newIndex,
                 const CameraRGBD& newCameraRgbd,
                 const std::string &newPathRGB,
                 const std::string &newPathD,
                 const Sophus::SE3d &absolutePose);

        VertexCG(int newIndex,
                 const CameraRGBD &newCameraRgbd,
                 const std::vector<KeyPoint2D> &newKeypoints,
                 const std::vector<float> &newDescriptors,
                 const std::vector<double> &newDepths,
                 const std::string &newPathRGB,
                 const std::string &newPathD);

        VertexCG(int newIndex,
                 const CameraRGBD &newCameraRgbd,
                 const keyPointsDepthDescriptor& keyPointsDepthDescriptor,
                 const std::string &newPathRGB,
                 const std::string &newPathD);

        std::string getPathRGBImage() const;

        std::string getPathDImage() const;

        const CameraRGBD &getCamera() const;

        int getIndex() const;

        std::vector<KeyPointInfo> getKeyPointInfoAllKeyPoints() const;
    };
}

#endif
