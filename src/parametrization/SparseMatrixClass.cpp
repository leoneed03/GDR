//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/SparseMatrixClass.h"

namespace gdr {

    SparseMatrixClass::SparseMatrixClass(const SparseMatrixd &sparseMatrixd) : sparseMatrixInner(sparseMatrixd) {}

    const SparseMatrixd &SparseMatrixClass::getSparseMatrixd() const {
        return sparseMatrixInner;
    }
}
