//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_ESSENTIALMATRIX_H
#define TEST_SIFTGPU_ESSENTIALMATRIX_H


#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD


#include <opencv2/opencv.hpp>
#include "VertexCG.h"


typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

struct transformationRtMatrix {
    MatrixX innerTranformationRtMatrix;
    const VertexCG &vertexFrom;
    const VertexCG &vertexTo;
    MatrixX R;
    MatrixX t;

    transformationRtMatrix(const MatrixX &newInnerEssentialMatrix, const VertexCG &newVertexFrom,
                           const VertexCG &newVertexTo);
};

#endif
