//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <sophus/se3.hpp>

#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"
#include "parametrization/Vectors3d.h"

namespace gdr {

    SparseMatrixd
    TranslationAverager::constructSparseMatrix(const std::vector<TranslationMeasurement> &relativeTranslations,
                                               const std::vector<SE3> &absolutePoses) {
        int numberOfAbsolutePoses = absolutePoses.size();
        int vectorDim = 3;
        std::vector<Tripletd> coefficients;

        for (int i = 0; i < relativeTranslations.size(); ++i) {

            const auto &relativeT = relativeTranslations[i];

            int posLeftPlus = relativeT.getIndexFromToBeTransformed();
            int posRightMinus = relativeT.getIndexToDestination();

            assert(posLeftPlus < posRightMinus);
            assert(posLeftPlus >= 0);
            assert(posRightMinus < numberOfAbsolutePoses);

            for (int counter = 0; counter < vectorDim; ++counter) {
                int posRow = 3 * i + counter;
                int posLeftCol = 3 * posLeftPlus + counter;
                int posRightCol = 3 * posRightMinus + counter;
                coefficients.emplace_back(Tripletd(posRow, posLeftCol, 1));
                coefficients.emplace_back(Tripletd(posRow, posRightCol, -1));
            }
        }
        SparseMatrixd systemSparseMatrix(vectorDim * relativeTranslations.size(), vectorDim * absolutePoses.size());
        systemSparseMatrix.setFromTriplets(coefficients.begin(), coefficients.end());

        assert(systemSparseMatrix.rows() / 3 == relativeTranslations.size());
        assert(systemSparseMatrix.cols() / 3 == absolutePoses.size());

        return systemSparseMatrix;
    }

    Vectors3d
    TranslationAverager::constructColumnTermB(const std::vector<TranslationMeasurement> &relativeTranslations,
                                              const std::vector<SE3> &absolutePoses) {
        std::vector<Eigen::Vector3d> b;
        b.reserve(relativeTranslations.size());

        for (const auto &relativeT: relativeTranslations) {
            b.emplace_back(absolutePoses[relativeT.getIndexToDestination()].getSO3().matrix() *
                           relativeT.getTranslation());
        }
        return Vectors3d(b);
    }

    Vectors3d
    TranslationAverager::findLeastSquaresSolution(const SparseMatrixd &anyMatrixA,
                                                  const Vectors3d &resultVector_b,
                                                  bool &success,
                                                  const SparseMatrixd &weightDiagonalMatrix,
                                                  bool useInitialGuess,
                                                  Vectors3d initialGuessX) {

        success = true;
        int dim = 3;

        assert(dim * resultVector_b.getSize() == anyMatrixA.rows());

        Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> lscg;

        lscg.compute(weightDiagonalMatrix * anyMatrixA);

        Eigen::VectorXd solutionL2;

        if (useInitialGuess) {
            assert(dim * initialGuessX.getSize() == anyMatrixA.cols());

            const Eigen::VectorXd &guessRaw = initialGuessX.getVectorRaw();
            solutionL2 = lscg.solveWithGuess(weightDiagonalMatrix * resultVector_b.getVectorRaw(), guessRaw);

        } else {
            solutionL2 = lscg.solve(weightDiagonalMatrix * resultVector_b.getVectorRaw());
        }


        if (lscg.info() != Eigen::Success) {
            success = false;
            std::cout << "NOT SUCCESS" << std::endl;
        }


        return Vectors3d(solutionL2);

    }

    SparseMatrixd
    TranslationAverager::getWeightMatrixRaw(const std::vector<double> &weights,
                                            double epsilonWeightMin) {

        int dim = 3;
        SparseMatrixd weightDiagonalMatrix(dim * weights.size(), dim * weights.size());
        std::vector<Tripletd> coefficients;
        coefficients.reserve(dim * weights.size());

        for (int i = 0; i < weights.size(); ++i) {
            for (int toDim = 0; toDim < dim; ++toDim) {

                int newRowColumnNumber = dim * i + toDim;
                //square the residual
                double newWeight = 1.0 / std::max(pow(weights[i], 2), epsilonWeightMin);
                coefficients.emplace_back(Tripletd(newRowColumnNumber, newRowColumnNumber, newWeight));
            }
        }
        weightDiagonalMatrix.setFromTriplets(coefficients.begin(), coefficients.end());

        return weightDiagonalMatrix;
    }

    Vectors3d
    TranslationAverager::IRLS(const SparseMatrixd &systemMatrix,
                              const Vectors3d &b,
                              SparseMatrixd &weightDiagonalMatrix,
                              const Vectors3d &translationsGuess,
                              bool &success,
                              int numOfIterations,
                              double epsilonIRLS) {

        success = false;

        Vectors3d bestSolutionAbsoluteTranslations(translationsGuess);
        Vectors3d prevResiduals = b - (SparseMatrixClass(systemMatrix) * translationsGuess);

        for (int iteration = 0; iteration < numOfIterations; ++iteration) {

            bool successCurrentIteration = true;
            Vectors3d currentSolutionAbsoluteTranslations =
                    findLeastSquaresSolution(systemMatrix,
                                             b,
                                             successCurrentIteration,
                                             weightDiagonalMatrix,
                                             true,
                                             bestSolutionAbsoluteTranslations);

            if (!successCurrentIteration) {

                return bestSolutionAbsoluteTranslations;
            }

            success = true;
            std::swap(bestSolutionAbsoluteTranslations, currentSolutionAbsoluteTranslations);

            Vectors3d residuals = b - (SparseMatrixClass(systemMatrix) * bestSolutionAbsoluteTranslations);

            if ((residuals.getVectorRaw() - prevResiduals.getVectorRaw()).isZero()) {
                std::cout << "IRLS CONVERGED, iteration: " << iteration << std::endl;

                return bestSolutionAbsoluteTranslations;
            }
            weightDiagonalMatrix = getWeightMatrixRaw(residuals.getVectorOfNorms(), epsilonIRLS);

            std::swap(prevResiduals, residuals);
        }

        return bestSolutionAbsoluteTranslations;
    }

    std::vector<TranslationMeasurement> TranslationAverager::getInversedTranslationMeasurements(
            const std::vector<TranslationMeasurement> &relativeTranslations,
            const std::vector<SE3> &absolutePoses) {

        std::vector<TranslationMeasurement> relativeTranslationsInversed;

        for (const auto &relativeTranslation: relativeTranslations) {
            int indexFrom = relativeTranslation.getIndexFromToBeTransformed();
            int indexTo = relativeTranslation.getIndexToDestination();
            assert(indexFrom < indexTo);

            Sophus::SE3d relativePoseFromTo;
            Sophus::SO3d relRot = absolutePoses[indexFrom].getSO3().inverse() *
                                  absolutePoses[indexTo].getSO3();

            relativePoseFromTo.setQuaternion(relRot.unit_quaternion());
            relativePoseFromTo.translation() = relativeTranslation.getTranslation();
            relativePoseFromTo = relativePoseFromTo.inverse();

            relativeTranslationsInversed.emplace_back(
                    TranslationMeasurement(relativePoseFromTo.translation(), indexFrom, indexTo));
        }

        return relativeTranslationsInversed;
    }


    Vectors3d
    TranslationAverager::recoverTranslationsIRLS(const std::vector<TranslationMeasurement> &relativeTranslations,
                                                 const std::vector<SE3> &absolutePoses,
                                                 const Vectors3d &absoluteTranslations,
                                                 bool &successIRLS,
                                                 int numOfIterations,
                                                 double epsilonWeightIRLS) {

        std::vector<TranslationMeasurement> relativeTranslationsInversed = getInversedTranslationMeasurements(
                relativeTranslations,
                absolutePoses);

        SparseMatrixd systemMatrix = constructSparseMatrix(relativeTranslationsInversed, absolutePoses);
        Vectors3d b = constructColumnTermB(relativeTranslationsInversed, absolutePoses);

        successIRLS = true;
        std::vector<double> weightsId(b.getSize(), 1.0);
        SparseMatrixd weightMatrixSparse = getWeightMatrixRaw(weightsId, epsilonWeightIRLS);

        return IRLS(systemMatrix, b,
                    weightMatrixSparse,
                    absoluteTranslations,
                    successIRLS,
                    numOfIterations, epsilonWeightIRLS);
    }

    Vectors3d
    TranslationAverager::recoverTranslations(const std::vector<TranslationMeasurement> &relativeTranslations,
                                             const std::vector<SE3> &absolutePoses) {

        std::vector<TranslationMeasurement> relativeTranslationsInversed = getInversedTranslationMeasurements(
                relativeTranslations,
                absolutePoses);

        SparseMatrixd systemMatrix = constructSparseMatrix(relativeTranslationsInversed, absolutePoses);
        Vectors3d b = constructColumnTermB(relativeTranslationsInversed, absolutePoses);
        bool success = true;
        std::vector<double> weightsId(b.getSize(), 1.0);

        // weight 1e016 is not used here because each weight is zero
        return findLeastSquaresSolution(systemMatrix, b,
                                        success,
                                        getWeightMatrixRaw(weightsId,
                                                           1e-6));
    }
}
