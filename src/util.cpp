//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/util.h"

MatrixX getSomeMatrix(int height, int width) {
    return MatrixX::Random(height, width);
}

Eigen::Matrix3d getRotationMatrixDouble(const MatrixX &m) {
    Eigen::Matrix3d resMatrix;

    assert(m.col() == 3);
    assert(m.rows() == 3);


    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            resMatrix.row(i)[j] = m.row(i)[j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(abs(resMatrix.row(i)[j] - m.row(i)[j]) < std::numeric_limits<double>::epsilon());
        }
    }
    return resMatrix;
}