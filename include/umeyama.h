//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GLOBALRECONSTRUCTIONRGBD_UMEYAMA_H
#define GLOBALRECONSTRUCTIONRGBD_UMEYAMA_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "pseudonames.h"


namespace gdr {

    Eigen::Matrix4d
    getTransformationMatrixUmeyamaLoRANSAC(const MatrixX &toBeTransormedPoints, const MatrixX &destinationPoints,
                                           const int numIterationsRansac,
                                           const int numOfElements, double inlierCoeff);
}
#endif
