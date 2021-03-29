//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/Vectors3d.h"
#include <iostream>

namespace gdr {
    Vectors3d::Vectors3d(const std::vector<Eigen::Vector3d> &vectors3d) {
        vectorsInner = Eigen::VectorXd(dim * vectors3d.size());
        size = vectors3d.size();
        for (int vectorIndex = 0; vectorIndex < vectors3d.size(); ++vectorIndex) {
            for (int toDim = 0; toDim < dim; ++toDim) {
                vectorsInner[vectorIndex * dim + toDim] = vectors3d[vectorIndex][toDim];
            }
        }
    }

    Vectors3d::Vectors3d(const std::vector<std::vector<double>> &vectors3d) {
        vectorsInner = Eigen::VectorXd(dim * vectors3d.size());
        size = vectors3d.size();
        for (int vectorIndex = 0; vectorIndex < vectors3d.size(); ++vectorIndex) {
            for (int toDim = 0; toDim < dim; ++toDim) {
                vectorsInner[vectorIndex * dim + toDim] = vectors3d[vectorIndex][toDim];
            }
        }
    }

    Vectors3d::Vectors3d(const Eigen::VectorXd &vectors3d) {
        assert(vectors3d.rows() % dim == 0);
        vectorsInner = vectors3d;
        size = vectors3d.rows() / 3;
    }

    Vectors3d::Vectors3d() {}

    const Eigen::VectorXd &Vectors3d::getVectorRaw() const {
        assert(vectorsInner.rows() % dim == 0);
        return vectorsInner;
    }


    size_t Vectors3d::getSize() const {
        return size;
    }


    std::vector<Eigen::Vector3d> Vectors3d::toVectorOfVectors(int indexFixedToZero) const {

        std::vector<Eigen::Vector3d> resultVectorOfVectors;
        resultVectorOfVectors.reserve(getSize());

        for (size_t index = 0; index < getSize(); ++index) {
            size_t startPos = dim * index;
            Eigen::Vector3d currentT = vectorsInner.block<dim, 1>(startPos, 0);
            assert(startPos >= 0 && startPos + dim - 1 < dim * getSize());
            resultVectorOfVectors.emplace_back(currentT);
        }

        return resultVectorOfVectors;
    }

    std::vector<double> Vectors3d::getVectorOfNorms() {
        std::vector<double> norms;
        norms.reserve(getSize());

        for (size_t index = 0; index < getSize(); ++index) {
            Eigen::Vector3d currentVector;
            for (int toDim = 0; toDim < dim; ++toDim) {
                assert(dim * index + toDim < dim * getSize() && dim * index + toDim >= 0);
                currentVector[toDim] = vectorsInner[dim * index + toDim];
            }
            norms.push_back(currentVector.norm());
        }
        assert(norms.size() == getSize());
        return norms;
    }


    gdr::Vectors3d operator*(const gdr::SparseMatrixClass &matrixOperator, const gdr::Vectors3d &vectors3D) {
        Eigen::VectorXd rawVectorArg = vectors3D.getVectorRaw();

        const SparseMatrixd &sparseMatrixRaw = matrixOperator.getSparseMatrixd();
        assert(sparseMatrixRaw.cols() == rawVectorArg.rows());
        Eigen::VectorXd result = sparseMatrixRaw * rawVectorArg;
        return gdr::Vectors3d(result);
    }

    Vectors3d operator-(const Vectors3d &vectors3dLeft, const Vectors3d &vectors3dRight) {
        Eigen::VectorXd result = vectors3dLeft.getVectorRaw() - vectors3dRight.getVectorRaw();
        return gdr::Vectors3d(result);
    }

    Eigen::Vector3d Vectors3d::operator[](size_t index) {
        Eigen::Vector3d translationByIndex = getVectorRaw().block<dim, 1>(index * dim, 0);
        return translationByIndex;
    }

    SparseMatrixClass::SparseMatrixClass(const SparseMatrixd &sparseMatrixd) : sparseMatrixInner(sparseMatrixd) {}


    const SparseMatrixd &SparseMatrixClass::getSparseMatrixd() const {
        return sparseMatrixInner;
    }
}

