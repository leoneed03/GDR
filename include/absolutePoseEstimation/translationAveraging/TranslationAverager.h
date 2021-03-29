//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TRANSLATIONAVERAGER_H
#define GDR_TRANSLATIONAVERAGER_H


#include "parametrization/SE3.h"

#include <Eigen/Eigen>
#include <vector>
#include <map>

#include "TranslationMeasurement.h"
#include "parametrization/Vectors3d.h"

namespace gdr {
    
    class TranslationAverager {

        static SparseMatrixd constructSparseMatrix(const std::vector<TranslationMeasurement> &relativeTranslations,
                                                   const std::vector<SE3> &absolutePoses);

        static Vectors3d
        constructColumnTermB(const std::vector<TranslationMeasurement> &relativeTranslations,
                             const std::vector<SE3> &absolutePoses);

        static Vectors3d
        findLeastSquaresSolution(const SparseMatrixd &anyMatrixA,
                                 const Vectors3d &b,
                                 bool &success,
                                 const SparseMatrixd &weightDiagonalMatrix,
                                 bool useInitialGuess = false,
                                 Vectors3d initialGuessX = Vectors3d());

        static SparseMatrixd getWeightMatrixRaw(const std::vector<double> &weights,
                                                double epsilonWeightMin);

        static Vectors3d
        IRLS(const SparseMatrixd &systemMatrix,
             const Vectors3d &b,
             SparseMatrixd &weightDiagonalMatrix,
             const Vectors3d& translationsGuess,
             bool &success,
             int numOfIterations,
             double epsilonIRLS);

    public:

        static Vectors3d
        recoverTranslationsIRLS(const std::vector<TranslationMeasurement> &relativeTranslations,
                                std::vector<SE3> &absolutePoses,
                                const Vectors3d& absoluteTranslations,
                                bool &successIRLS,
                                int numOfIterations = 5,
                                double epsilonIRLS = 1e-6);

        static Vectors3d
        recoverTranslations(const std::vector<TranslationMeasurement> &relativeTranslations,
                            const std::vector<SE3> &absolutePoses,
                            double epsilonIRLSWeightMin = 1e-6);
    };
}

#endif
