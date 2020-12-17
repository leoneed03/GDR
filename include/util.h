//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_UTIL_H
#define TEST_SIFTGPU_UTIL_H

#include <Eigen/Eigen>

typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

MatrixX getSomeMatrix(int h, int w);

Eigen::Matrix3d getRotationMatrixDouble(const MatrixX &m);

#endif
