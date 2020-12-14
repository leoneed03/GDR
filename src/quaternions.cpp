//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/quaternions.h"

#include <limits>
#include <iostream>

MatrixX copyMatrix(Eigen::Quaterniond &quat) {
    assert(quat.size() == 4);
    auto matrixQuat = quat.toRotationMatrix();
    assert(matrixQuat.rows() == 3);

    MatrixX result = getSomeMatrix(matrixQuat.rows(), matrixQuat.cols());
    for (int i = 0; i < 3; ++i) {
        assert(matrixQuat[i].size() == 3);
        for (int j = 0; j < 3; ++j) {
            result.row(i)[j] = matrixQuat.row(i)[j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            assert(abs(result.row(i)[j] - matrixQuat.row(i)[j]) < std::numeric_limits<float>::epsilon());
        }
    }
    return result;
}

std::vector<MatrixX> getRotationsFromQuaternionVector(const std::vector<std::vector<double>> &quats) {
    std::vector<MatrixX> resultMatrices;
    int count0 = 0;
    for (const auto &quat: quats) {
        ++count0;
        Eigen::Quaterniond quatd(quat.data());
        auto matrixQuat = copyMatrix(quatd);
        resultMatrices.push_back(matrixQuat);
    }
    std::cout << "processed" << std::endl;
    return resultMatrices;
}
