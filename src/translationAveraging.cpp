//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>

#include "translationAveraging.h"
#include "printer.h"
#include "Vectors3d.h"

namespace gdr {
    SparseMatrixd
    translationAverager::constructSparseMatrix(const std::vector<translationMeasurement> &relativeTranslations,
                                               const std::vector<Eigen::Matrix4d> &absolutePoses) {
        int numberOfAbsolutePoses = absolutePoses.size();
        int vectorDim = 3;
        std::vector<Tripletd> coefficients;

        for (int i = 0; i < relativeTranslations.size(); ++i) {

            const auto &relativeT = relativeTranslations[i];
            int posLeftPlus = relativeT.indexFrom;
            int posRightMinus = relativeT.indexTo;
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

        PRINT_PROGRESS("Sparse A is " << systemSparseMatrix.rows() / 3 << " * " << systemSparseMatrix.cols() / 3
                                      << std::endl);
        assert(systemSparseMatrix.rows() / 3 == relativeTranslations.size());
        assert(systemSparseMatrix.cols() / 3 == absolutePoses.size());

        return systemSparseMatrix;
    }

    Vectors3d
    translationAverager::constructColumnTermB(const std::vector<translationMeasurement> &relativeTranslations,
                                              const std::vector<Eigen::Matrix4d> &absolutePoses) {
        std::vector<Eigen::Vector3d> b;
        b.reserve(relativeTranslations.size());
        for (const auto &relativeT: relativeTranslations) {
            b.push_back(absolutePoses[relativeT.indexTo].block<3, 3>(0, 0) * relativeT.translation3d);
        }
        return Vectors3d(b);
    }

    Vectors3d
    translationAverager::findLeastSquaresSolution(const SparseMatrixd &anyMatrixA,
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
        Eigen::VectorXd guessRaw = initialGuessX.getVectorRaw();


        auto SymMatrix = anyMatrixA.transpose() * weightDiagonalMatrix * anyMatrixA;

        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
        cg.compute(SymMatrix);
        if (cg.info() != Eigen::Success) {
            success = false;
        }
        Eigen::VectorXd b = resultVector_b.getVectorRaw();


        Eigen::VectorXd freeTermToSolveB = anyMatrixA.transpose() * weightDiagonalMatrix * b;

        Eigen::VectorXd x;
        if (!useInitialGuess) {
            x = cg.solve(freeTermToSolveB);
        } else {
            x = cg.solveWithGuess(freeTermToSolveB, guessRaw);
        }

        std::cout << "A is " << anyMatrixA.rows() / 3 << " * " << anyMatrixA.cols() / 3 << std::endl;
        std::cout << "A^T is " << anyMatrixA.transpose().rows() / 3 << " * " << anyMatrixA.transpose().cols() / 3
                  << std::endl;
        std::cout << "System Matrix Symmetric is " << SymMatrix.rows() / 3 << " * " << SymMatrix.cols() / 3
                  << std::endl;
        std::cout << "Free term is " << freeTermToSolveB.rows() / 3 << std::endl;
        std::cout << "Solution is " << x.rows() / 3 << std::endl;


        if (cg.info() != Eigen::Success) {
            success = false;
        }

        assert(x.rows() % 3 == 0);
        std::cout << "rows after solution " << x.rows() / 3 << std::endl;
        Vectors3d solution(x);


        if (success) {
            std::cout << "SUCCESS!!" << std::endl;
        }
//        std::cout << "Matrix:\n" << anyMatrixA << std::endl;
        return solution;

    }

    SparseMatrixd
    translationAverager::getWeightMatrixRaw(const std::vector<double> &weights,
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

//
//    Vectors3d subVectorOfTranslations(const Vectors3d &leftVector,
//                                      const Vectors3d &rightVector) {
//
//        assert(leftVector.size() == rightVector.size());
//        Vectors3d result;
//        result.reserve(rightVector.size());
//
//        for (int i = 0; i < leftVector.size(); ++i) {
//            result.emplace_back(leftVector[i] - rightVector[i]);
//        }
//
//        return result;
//    }

//    std::vector<double> getVectorOfNorms(const Vectors3d &objectVector) {
//
//        std::vector<double> norms;
//        norms.reserve(objectVector.size());
//        for (auto &t: objectVector) {
//            norms.emplace_back(t.norm());
//        }
//
//        return norms;
//    }

//    Vectors3d applyOperator(const SparseMatrixd &matrix,
//                            const Vectors3d &rightVector) {
//
//        int dim = 3;
//        Eigen::VectorXd rawVector(dim * rightVector.size());
//        for (int i = 0; i < rightVector.size(); ++i) {
//            for (int toDim = 0; toDim < dim; ++toDim) {
//                rawVector[dim * i + toDim] = rightVector[i][toDim];
//            }
//        }
//        Eigen::VectorXd resultRaw = matrix * rawVector;
////        int outputVectorLength = resultRaw.rows() / dim;
////        Vectors3d result(outputVectorLength);
////        assert(resultRaw.rows() % dim == 0);
////        for (int i = 0; i < outputVectorLength; ++i) {
////            for (int toDim = 0; toDim < dim; ++toDim) {
////                result[i][toDim] = resultRaw[dim * i + toDim];
////            }
////        }
//
//
//        return Vectors3d(resultRaw);
//    }


    Vectors3d
    translationAverager::IRLS(const SparseMatrixd &systemMatrix,
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
                    findLeastSquaresSolution(systemMatrix, b, successCurrentIteration, weightDiagonalMatrix, true,
                                             currentSolutionAbsoluteTranslations);
            Vectors3d residuals =
                    b - (SparseMatrixClass(systemMatrix) * currentSolutionAbsoluteTranslations);
            weightDiagonalMatrix = getWeightMatrixRaw(residuals.getVectorOfNorms(), epsilonIRLS);

        }
        return currentSolutionAbsoluteTranslations;
    }

    Vectors3d
    translationAverager::recoverTranslationsIRLS(const std::vector<translationMeasurement> &relativeTranslations,
                                                 std::vector<Eigen::Matrix4d> &absolutePosesGuess,
                                                 const Vectors3d &absoluteTranslations,
                                                 bool &successIRLS,
                                                 int numOfIterations,
                                                 double epsilonIRLS) {

        int dim = 3;
        SparseMatrixd systemMatrix = constructSparseMatrix(relativeTranslations, absolutePosesGuess);
        Vectors3d b = constructColumnTermB(relativeTranslations, absolutePosesGuess);
//        Vectors3d translationsGuess;
//        translationsGuess.reserve(absolutePosesGuess.size());
//
//        for (const auto &absolutePose: absolutePosesGuess) {
//            translationsGuess.emplace_back(absolutePose.block<3, 1>(0, 3));
//        }

        successIRLS = true;
        std::vector<double> weightsId(b.getSize(), 1.0);
        SparseMatrixd weightMatrixSparse = getWeightMatrixRaw(weightsId, epsilonIRLS);
        return IRLS(systemMatrix, b, weightMatrixSparse, absoluteTranslations, successIRLS, numOfIterations,
                    epsilonIRLS);
    }

    Vectors3d
    translationAverager::recoverTranslations(const std::vector<translationMeasurement> &relativeTranslations,
                                             const std::vector<Eigen::Matrix4d> &absolutePoses,
                                             double epsilonIRLSWeightMin) {

        SparseMatrixd systemMatrix = constructSparseMatrix(relativeTranslations, absolutePoses);
        Vectors3d b = constructColumnTermB(relativeTranslations, absolutePoses);
        bool success = true;
        std::vector<double> weightsId(b.getSize(), 1.0);

        return findLeastSquaresSolution(systemMatrix, b, success, getWeightMatrixRaw(weightsId, epsilonIRLSWeightMin));
    }
}
