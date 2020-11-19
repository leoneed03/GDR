#pragma once

#ifndef TEST_SIFTGPU_ESSENTIALMATRIX_H
#define TEST_SIFTGPU_ESSENTIALMATRIX_H


#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/LU> // required for MatrixBase::determinant
#include <Eigen/SVD> // required for SVD


#include <opencv2/opencv.hpp>
#include "vertexCG.h"



typedef typename Eigen::internal::traits<Eigen::MatrixXd>::Scalar Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

typedef struct essentialMatrix {
    MatrixX innerEssentialMatrix;
    const vertexCG& vertexFrom;
    const vertexCG& vertexTo;
    MatrixX R;
    MatrixX t;
    essentialMatrix(const MatrixX& newInnerEssentialMatrix, const vertexCG& newVertexFrom, const vertexCG& newVertexTo, const MatrixX& newR, const MatrixX& newT);
} essentialMatrix;

#endif
