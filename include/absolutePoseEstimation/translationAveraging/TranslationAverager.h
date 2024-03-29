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
                                                   const std::vector<SE3> &absolutePoses,
                                                   int indexPoseFixed);

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
             const Vectors3d &translationsGuess,
             bool &success,
             int numOfIterations,
             double epsilonIRLS);

        static std::vector<TranslationMeasurement> getInversedTranslationMeasurements(
                const std::vector<TranslationMeasurement> &relativeTranslations,
                const std::vector<SE3> &absolutePoses);

    public:

        /**
         * Compute IRLS solution for sparse linear system
         *
         * @param relativeTranslations contains given relative translations between poses
         * @param absolutePoses contains precomputed SE3 poses where SO3 rotations are already fixed
         *      and translations are not currently utilized
         * @param absoluteTranslations initial solution guess
         * @param indexPoseFixed is the number of pose with zero coordinates
         * @param successIRLS[out] is true if IRLS did converge
         * @param numOfIterations max number of iterations
         * @param epsilonWeightIRLS is a value w such that all weights in weight matrix are less than 1/w
         * @returns IRLS solution
         */
        static Vectors3d
        recoverTranslationsIRLS(const std::vector<TranslationMeasurement> &relativeTranslations,
                                const std::vector<SE3> &absolutePoses,
                                const Vectors3d &absoluteTranslations,
                                int indexPoseFixed,
                                bool &successIRLS,
                                int numOfIterations = 5,
                                double epsilonWeightIRLS = 1e-6);

        /**
         * Compute L2-solution for sparse linear problem
         *
         * @param relativeTranslations contains given relative translations between poses
         * @param absolutePoses contains precomputed SE3 poses where SO3 rotations are already fixed
         *      and translations are not currently utilized
         *
         * @returns L2 solution
         */
        static Vectors3d
        recoverTranslations(const std::vector<TranslationMeasurement> &relativeTranslations,
                            const std::vector<SE3> &absolutePoses,
                            int indexPoseFixed);
    };
}

#endif
