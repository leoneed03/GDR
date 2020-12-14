//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_VERTEXCG_H
#define TEST_SIFTGPU_VERTEXCG_H

#include <vector>
#include <iostream>
#include <SiftGPU.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU>
#include <Eigen/SVD>

#include "features.h"
#include "quaternions.h"

struct keypointWithDepth {
    SiftGPU::SiftKeypoint keypoint;
    double depth;
    std::vector<float> descriptors;

    keypointWithDepth(SiftGPU::SiftKeypoint newKeypoint, double newDepth, const std::vector<float> &newDescriptors);
};


typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

struct VertexCG {

    int index;
    MatrixX absoluteRotationTranslation;
    std::vector<SiftGPU::SiftKeypoint> keypoints;
    std::vector<float> descriptors;
    std::vector<double> depths;
    std::string pathToRGBimage;
    std::string pathToDimage;
    std::vector<keypointWithDepth> keypointsWithDepths;
    int heightMirrorParameter = 480;
    int widthMirrorParameter = 640;

    void setRotation(const MatrixX &rotation);

    VertexCG(int newIndex,
             const std::vector<keypointWithDepth> &newKeypointsWithDepths,
             const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
             const std::vector<float> &newDescriptors,
             const std::vector<double> &newDepths,
             const std::string &newPathRGB,
             const std::string &newPathD);
};

#endif
