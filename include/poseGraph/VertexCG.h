//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_VERTEXCG_H
#define GDR_VERTEXCG_H

#include <vector>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU>
#include <Eigen/SVD>

#include <sophus/se3.hpp>

#include "keyPoints/KeyPointInfo.h"
#include "parametrization/SE3.h"
#include "parametrization/SO3.h"
#include "parametrization/cameraRGBD.h"
#include "keyPoints/KeyPointsDepthDescriptor.h"


namespace gdr {

    struct VertexCG {

        CameraRGBD cameraRgbd;
        int index;
        int initialIndex;
        SE3 absolutePose;
        std::vector<KeyPoint2D> keypoints;
        std::vector<float> descriptors;
        std::vector<double> depths;
        std::string pathToRGBimage;
        std::string pathToDimage;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setIndex(int newIndex);

        void setRotation(const SO3 &orientation);

        void setRotation(const Sophus::SO3d &orientation);

        void setRotation(const Eigen::Matrix3d &orientation);

        void setRotation(const Eigen::Quaterniond &orientationQuatd);

        void setTranslation(const Eigen::Vector3d &translation);

        void setRotationTranslation(const Eigen::Matrix4d &eigenRt);

        void setRotationTranslation(const Sophus::SE3d &sophusRt);

        void setAbsolutePoseSE3(const SE3 &absolutePose);

        const std::vector<KeyPoint2D>& getKeyPoints() const;

        const std::vector<float>& getDescriptors() const;

        const std::vector<double>& getDepths() const;

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
