//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>

#include "translationAveraging.h"

namespace gdr {
    translationAverager::SparseMatrixd
    translationAverager::constructSparseMatrix(const std::vector<translationMeasurement> &relativeTranslations,
                                               const std::vector<Eigen::Matrix4d> &absolutePoses) {
        int numberOfAbsolutePoses = absolutePoses.size();
//        std::cout << "should be little " << absolutePoses.size() << std::endl;
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

        std::cout << "Sparse A is " << systemSparseMatrix.rows() / 3 << " * " << systemSparseMatrix.cols() / 3 << std::endl;
        assert(systemSparseMatrix.rows() / 3 == relativeTranslations.size());
        assert(systemSparseMatrix.cols() / 3 == absolutePoses.size());

        return systemSparseMatrix;
    }

    std::vector<Eigen::Vector3d>
    translationAverager::constructColumnTermB(const std::vector<translationMeasurement> &relativeTranslations,
                                              const std::vector<Eigen::Matrix4d> &absolutePoses) {
        std::vector<Eigen::Vector3d> b;
        b.reserve(relativeTranslations.size());
        for (const auto &relativeT: relativeTranslations) {
            b.push_back(absolutePoses[relativeT.indexTo].block<3, 3>(0, 0) * relativeT.translation3d);
        }
        return b;
    }

    std::vector<Eigen::Vector3d>
    translationAverager::findLeastSquaresSolution(const translationAverager::SparseMatrixd &anyMatrixA,
                                                  const std::vector<Eigen::Vector3d> &resultVector_b,
                                                  bool &success,
                                                  std::vector<Eigen::Vector3d> initialGuessX) {

        success = true;
        assert(3 * resultVector_b.size() == anyMatrixA.rows());

        bool useInitialGuess = !initialGuessX.empty();

        if (useInitialGuess) {
            assert(3 * initialGuessX.size() == anyMatrixA.cols());
        }


        auto SymMatrix = anyMatrixA.transpose() * anyMatrixA;

        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
        cg.compute(SymMatrix);
        if (cg.info() != Eigen::Success) {
            success = false;
        }
        Eigen::VectorXd b(3 * resultVector_b.size());
        for (int i = 0; i < resultVector_b.size(); ++i) {
            b.block<3, 1>(3 * i, 0) = resultVector_b[i];
        }


        Eigen::VectorXd freeTermToSolveB = anyMatrixA.transpose() * b;
        Eigen::VectorXd x = cg.solve(freeTermToSolveB);


        std::cout << "A is " << anyMatrixA.rows() / 3 << " * " << anyMatrixA.cols() / 3 << std::endl;
        std::cout << "A^T is " << anyMatrixA.transpose().rows() / 3 << " * " << anyMatrixA.transpose().cols() / 3 << std::endl;
        std::cout << "System Matrix Symmetric is " << SymMatrix.rows() / 3 << " * " << SymMatrix.cols() / 3 << std::endl;
        std::cout << "Free term is " << freeTermToSolveB.rows() / 3 << std::endl;
        std::cout << "Solution is " << x.rows() / 3 << std::endl;


        if (cg.info() != Eigen::Success) {
            success = false;
        }

        assert(x.rows() % 3 == 0);
        std::cout << "rows after solution " << x.rows() / 3 << std::endl;
        std::vector<Eigen::Vector3d> solution(x.rows() / 3);
        for (int i = 0; i < solution.size(); ++i) {
            solution[i] = x.block<3, 1>(3 * i, 0);
        }

        if (success) {
            std::cout << "SUCCESS!!" << std::endl;
        }
//        std::cout << "Matrix:\n" << anyMatrixA << std::endl;
        return solution;

    }

    std::vector<Eigen::Vector3d>
    translationAverager::recoverTranslations(const std::vector<translationMeasurement> &relativeTranslations,
                                             const std::vector<Eigen::Matrix4d> &absolutePoses) {

        SparseMatrixd systemMatrix = constructSparseMatrix(relativeTranslations, absolutePoses);
        std::vector<Eigen::Vector3d> b = constructColumnTermB(relativeTranslations, absolutePoses);
        bool success = true;

        return findLeastSquaresSolution(systemMatrix, b, success);
    }
}
