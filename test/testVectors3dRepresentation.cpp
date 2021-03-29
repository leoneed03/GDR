//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>

#include "parametrization/Vectors3d.h"

TEST(testVectors3d, simpleTestConstructorId) {

    int dim = 3;
    std::vector<double> points2 = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};

    int rawSize = points2.size();
    int numVectors3d = rawSize / dim;

    gdr::SparseMatrixd sparseMatrixSquare(rawSize, rawSize);
    sparseMatrixSquare.setIdentity();
    gdr::SparseMatrixClass sparseMatrixClass(sparseMatrixSquare);
    Eigen::VectorXd translations2(rawSize);
    for (int i = 0; i < points2.size(); ++i) {
        translations2[i] = points2[i];
    }

    gdr::Vectors3d vectors3D(translations2);
    gdr::Vectors3d result = sparseMatrixClass * vectors3D;

    std::cout << result.getVectorRaw() << std::endl;

    ASSERT_EQ(result.getSize(), numVectors3d);

    for (int i = 0; i < numVectors3d; ++i) {
        auto vectorByIndex = result[i];
        for (int index = 0; index < dim; ++index) {
            ASSERT_DOUBLE_EQ(vectorByIndex[index], points2[i * dim + index]);
        }
    }
}


TEST(testVectors3d, simpleTestSimpleProduct) {

    int dim = 3;
    std::vector<double> points2 = {11, 12, 13, 21, 22, 23, 31, 32, 33};

    int rawSize = points2.size();
    int numVectors3d = rawSize / dim;
    double coeffMultiply = 2.0;



    std::vector<gdr::Tripletd> coefficients;

    for (int i = 0; i < dim; ++i) {
        coefficients.emplace_back(gdr::Tripletd(i, dim + i, 1));
    }

    gdr::SparseMatrixd sparseMatrixSquare(dim, rawSize);
    sparseMatrixSquare.setFromTriplets(coefficients.begin(), coefficients.end());
    sparseMatrixSquare *= coeffMultiply;

    gdr::SparseMatrixClass sparseMatrixClass(sparseMatrixSquare);
    Eigen::VectorXd translations2(rawSize);
    for (int i = 0; i < points2.size(); ++i) {
        translations2[i] = points2[i];
    }

    gdr::Vectors3d vectors3D(translations2);
    gdr::Vectors3d result = sparseMatrixClass * vectors3D;

    std::cout << "======================================" << std::endl;
    std::cout << sparseMatrixSquare << std::endl;
    ASSERT_EQ(result.getSize(), 1);
    for (int i = 0; i < coefficients.size() / dim; ++i) {
        auto vectorByIndex = result[i];
        for (int index = 0; index < dim; ++index) {
            ASSERT_DOUBLE_EQ(vectorByIndex[index], coeffMultiply * points2[dim + index]);
        }
    }
    std::cout << result.getVectorRaw() << std::endl;
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}