//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SPARSEMATRIXCLASS_H
#define GDR_SPARSEMATRIXCLASS_H

#include <Eigen/Eigen>

namespace gdr {

    typedef Eigen::SparseMatrix<double> SparseMatrixd;
    typedef Eigen::Triplet<double> Tripletd;

    class SparseMatrixClass {

        Eigen::SparseMatrix<double> sparseMatrixInner;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SparseMatrixClass(const SparseMatrixd &sparseMatrixd);

        const SparseMatrixd &getSparseMatrixd() const;
    };
}

#endif
