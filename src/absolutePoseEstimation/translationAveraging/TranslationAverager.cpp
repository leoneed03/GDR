//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>

#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"
#include "Vectors3d.h"

#include <sophus/se3.hpp>

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
                coefficients.push_back(Tripletd(posRow, posLeftCol, 1));
                coefficients.push_back(Tripletd(posRow, posRightCol, -1));
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

        if (useInitialGuess) {
            assert(dim * initialGuessX.getSize() == anyMatrixA.cols());
        }
        const Eigen::VectorXd &guessRaw = initialGuessX.getVectorRaw();


        auto SymMatrix = anyMatrixA.transpose() * weightDiagonalMatrix * anyMatrixA;

        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
        cg.compute(SymMatrix);
        if (cg.info() != Eigen::Success) {
            success = false;
        }
        assert(cg.info() == Eigen::Success);
        const Eigen::VectorXd &b = resultVector_b.getVectorRaw();


        Eigen::VectorXd freeTermToSolveB = anyMatrixA.transpose() * weightDiagonalMatrix * b;

        Eigen::VectorXd x;

        if (!useInitialGuess) {
            x = cg.solve(freeTermToSolveB);
        } else {
            x = cg.solveWithGuess(freeTermToSolveB, guessRaw);
        }

        //TODO: catch unsuccessful steps
        if (cg.info() != Eigen::Success) {
            success = false;
        }

        assert(x.rows() % 3 == 0);
        Vectors3d solution(x);

        return solution;

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

        Vectors3d currentSolutionAbsoluteTranslations = translationsGuess;

        for (int iteration = 0; iteration < numOfIterations; ++iteration) {

            bool successCurrentIteration = true;
            currentSolutionAbsoluteTranslations =
                    findLeastSquaresSolution(systemMatrix,
                                             b,
                                             successCurrentIteration,
                                             weightDiagonalMatrix,
                                             true,
                                             currentSolutionAbsoluteTranslations);
            Vectors3d residuals = b - (SparseMatrixClass(systemMatrix) * currentSolutionAbsoluteTranslations);
            weightDiagonalMatrix = getWeightMatrixRaw(residuals.getVectorOfNorms(), epsilonIRLS);
        }

        return currentSolutionAbsoluteTranslations;
    }


    Vectors3d
    TranslationAverager::recoverTranslationsIRLS(const std::vector<TranslationMeasurement> &relativeTranslations,
                                                 std::vector<SE3> &absolutePosesGuess,
                                                 const Vectors3d &absoluteTranslations,
                                                 bool &successIRLS,
                                                 int numOfIterations,
                                                 double epsilonIRLS) {

        const int dim = 3;
        std::vector<TranslationMeasurement> newRelativeTranslations;

        for (int i = 0; i < relativeTranslations.size(); ++i) {
            int indexFrom = relativeTranslations[i].getIndexFromToBeTransformed();
            int indexTo = relativeTranslations[i].getIndexToDestination();

            assert(indexFrom < indexTo);
            Sophus::SE3d relativePoseFromTo;
            Sophus::SO3d relRot = absolutePosesGuess[indexFrom].getSO3().inverse() *
                                     absolutePosesGuess[indexTo].getSO3();
            relativePoseFromTo.setQuaternion(relRot.unit_quaternion());
            relativePoseFromTo.translation() = relativeTranslations[i].getTranslation();
            relativePoseFromTo = relativePoseFromTo.inverse();
            newRelativeTranslations.push_back(
                    TranslationMeasurement(relativePoseFromTo.translation(), indexFrom, indexTo));
        }

        SparseMatrixd systemMatrix = constructSparseMatrix(newRelativeTranslations, absolutePosesGuess);
        Vectors3d b = constructColumnTermB(newRelativeTranslations, absolutePosesGuess);

        successIRLS = true;
        std::vector<double> weightsId(b.getSize(), 1.0);
        SparseMatrixd weightMatrixSparse = getWeightMatrixRaw(weightsId, epsilonIRLS);
        return IRLS(systemMatrix, b, weightMatrixSparse, absoluteTranslations, successIRLS, numOfIterations,
                    epsilonIRLS);
    }

    Vectors3d
    TranslationAverager::recoverTranslations(const std::vector<TranslationMeasurement> &relativeTranslations,
                                             const std::vector<SE3> &absolutePoses,
                                             double epsilonIRLSWeightMin) {

        const int dim = 3;
        std::vector<TranslationMeasurement> newRelativeTranslations;
        for (int i = 0; i < relativeTranslations.size(); ++i) {
            int indexFrom = relativeTranslations[i].getIndexFromToBeTransformed();
            int indexTo = relativeTranslations[i].getIndexToDestination();
            assert(indexFrom < indexTo);
            Sophus::SE3d relativePoseFromTo;
            Sophus::SO3d relRot = absolutePoses[indexFrom].getSO3().inverse() *
                                     absolutePoses[indexTo].getSO3();
            relativePoseFromTo.setQuaternion(relRot.unit_quaternion());
            relativePoseFromTo.translation() = relativeTranslations[i].getTranslation();
            relativePoseFromTo = relativePoseFromTo.inverse();
            newRelativeTranslations.emplace_back(
                    TranslationMeasurement(relativePoseFromTo.translation(), indexFrom, indexTo));
        }

        SparseMatrixd systemMatrix = constructSparseMatrix(newRelativeTranslations, absolutePoses);
        Vectors3d b = constructColumnTermB(newRelativeTranslations, absolutePoses);
        bool success = true;
        std::vector<double> weightsId(b.getSize(), 1.0);

        return findLeastSquaresSolution(systemMatrix, b, success, getWeightMatrixRaw(weightsId, epsilonIRLSWeightMin));
    }
}
