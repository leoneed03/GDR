//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_VECTORS3D_H
#define GDR_VECTORS3D_H

#include <Eigen/Eigen>

#include "parametrization/SparseMatrixClass.h"

namespace gdr {

    class Vectors3d {

        size_t size = 0;
        static const int dim = 3;
        Eigen::VectorXd vectorsInner;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<double> getVectorOfNorms();

        Vectors3d();

        Vectors3d(const std::vector<Eigen::Vector3d> &vectors3d);

        Vectors3d(const std::vector<std::vector<double>> &vectors3d);

        Vectors3d(const Eigen::VectorXd &vectors3d);

        std::vector<Eigen::Vector3d> toVectorOfVectors(int indexFixedToZero = 0) const;

        const Eigen::VectorXd &getVectorRaw() const;

        size_t getSize() const;

        friend Vectors3d operator*(const SparseMatrixClass &matrixOperator, const Vectors3d &vectors3D);

        friend Vectors3d operator-(const Vectors3d &vectors3dLeft, const Vectors3d &vectors3dRight);

        Eigen::Vector3d operator[](size_t index);
    };
}

#endif
