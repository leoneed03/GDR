//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "CorrespondenceGraph.h"

TEST(testUmeyamaRansac, allInliers) {

    int numOfPoints = 100;
    Eigen::Matrix3d rotationMatrix;
    std::vector<double> angles = {30, 50, -87};

    rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d transformationMatrix;
    transformationMatrix.setIdentity();
    transformationMatrix.block(0, 0, 3, 3) = rotationMatrix;

    std::vector<double> translation = {3, 0.5, -0.5};
    for (int i = 0; i < 3; ++i) {
        transformationMatrix.col(3)[i] = translation[i];
    }

    Eigen::Matrix4Xd src(4, numOfPoints);
    src.setOnes();
    src.block(0, 0, 3, src.cols()).setRandom();

    Eigen::Matrix4Xd dst = transformationMatrix * src;

    std::cout << src << std::endl;
    bool estimationSuccess = true;
    double maxErrorTreshold = 0.05;
    Eigen::Matrix4d umeyamaTransformation = gdr::getTransformationMatrixUmeyamaLoRANSAC(
            src, dst, 50, src.cols(), 0.9, estimationSuccess, maxErrorTreshold
    );

    auto error = (dst - umeyamaTransformation * src).squaredNorm();

    auto mse = 1.0 * error / src.cols();

    ASSERT_LE(mse, 3 * std::numeric_limits<double>::epsilon());
}


TEST(testUmeyamaRansac, Inliers90percent) {

    int numOfPoints = 100;
    double outlierCoeff = 0.1;
    int numOutliers = 100 * outlierCoeff;
    Eigen::Matrix3d rotationMatrix;
    std::vector<double> angles = {10, 5, 70};

    rotationMatrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());

    Eigen::Matrix4d transformationMatrix;
    transformationMatrix.setIdentity();
    transformationMatrix.block(0, 0, 3, 3) = rotationMatrix;
    std::vector<double> translation = {3, 0.5, -0.5};
    for (int i = 0; i < 3; ++i) {
        transformationMatrix.col(3)[i] = translation[i];
    }

    Eigen::Matrix4Xd src(4, numOfPoints);
    src.setOnes();
    src.block(0, 0, 3, src.cols()).setRandom();

    Eigen::Matrix4Xd outliers = Eigen::Matrix4Xd::Random(4, numOutliers);
    outliers.setOnes();
    outliers.block(0, 0, 3, outliers.cols()).setRandom();


    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    std::uniform_int_distribution<> distrib(0, numOfPoints - 1);

    for (int i = 0; i < numOutliers; ++i) {
        int pos = distrib(randomNumberGenerator);
        src.col(pos) = outliers.col(i);
    }

    Eigen::Matrix4Xd dst = transformationMatrix * src;

    bool estimationSuccess = true;
    double maxErrorTreshold = 0.05;

    Eigen::Matrix4d umeyamaTransformation = gdr::getTransformationMatrixUmeyamaLoRANSAC(
            src, dst, 50, src.cols(), 0.8, estimationSuccess, maxErrorTreshold
    );

    ASSERT_TRUE(estimationSuccess);

    Eigen::Matrix4Xd afterTransformation = umeyamaTransformation * src;

    std::vector<double> errors;
    for (int i = 0; i < afterTransformation.cols(); ++i) {
        errors.push_back((afterTransformation.col(i) - dst.col(i)).squaredNorm());
    }
    std::sort(errors.begin(), errors.end());
    errors.resize(numOfPoints - numOutliers);

    double mse = 0;
    for (const auto &e: errors) {
        mse += e;
    }
    mse /= errors.size();

    ASSERT_LE(mse, 3 * std::numeric_limits<double>::epsilon());
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}