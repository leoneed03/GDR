//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef TEST_SIFTGPU_QUATERNIONS_H
#define TEST_SIFTGPU_QUATERNIONS_H

#include "util.h"

MatrixX copyMatrix(Eigen::Quaterniond &quat);

std::vector<MatrixX> getRotationsFromQuaternionVector(const std::vector<std::vector<double>> &quats);

#endif
