//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TRANSLATIONAVERAGING_H
#define GDR_TRANSLATIONAVERAGING_H

#include <Eigen/Eigen>
#include <vector>
#include <map>

#include "translationMeasurement.h"

namespace gdr {
    struct translationAverager {


        typedef Eigen::SparseMatrix<double> SparseMatrixd;
        typedef Eigen::Triplet<double> Tripletd;

        static SparseMatrixd constructSparseMatrix(const std::vector<translationMeasurement> &relativeTranslations,
                                                   const std::vector<Eigen::Matrix4d> &absolutePoses);
        static std::vector<Eigen::Vector3d> constructColumnTermB(const std::vector<translationMeasurement> &relativeTranslations,
                                                                 const std::vector<Eigen::Matrix4d> &absolutePoses);
        static std::vector<Eigen::Vector3d>
        findLeastSquaresSolution(const SparseMatrixd &anyMatrixA,
                                 const std::vector<Eigen::Vector3d> &b,
                                 bool& success,
                                 std::vector<Eigen::Vector3d> initialGuessX = std::vector<Eigen::Vector3d>());

        static std::vector<Eigen::Vector3d>
        recoverTranslations(const std::vector<translationMeasurement> &relativeTranslations,
                            const std::vector<Eigen::Matrix4d> &absolutePoses);
    };
}

#endif
