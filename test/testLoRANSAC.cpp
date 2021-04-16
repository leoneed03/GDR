//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "parametrization/SO3.h"

#include "relativePoseEstimators/EstimatorRobustLoRANSAC.h"

Eigen::MatrixXd getRandomMatrixLowRowOnes(int numberOfPoints, double maxValue, int dim = 3) {

    assert(dim >= 2);

    assert(numberOfPoints > 0);

    Eigen::MatrixXd destinationPoints(dim + 1, numberOfPoints);

    destinationPoints.setRandom();
    destinationPoints /= destinationPoints.maxCoeff();
    destinationPoints *= maxValue;

    assert(std::abs(maxValue - destinationPoints.maxCoeff()) < std::numeric_limits<double>::epsilon());

    if (dim >= 3) {
        destinationPoints.block(dim, 0, 1, numberOfPoints).setOnes();
    }

    return destinationPoints;
}

void setOutliers(Eigen::Matrix4Xd &destination,
                 Eigen::Matrix4Xd &toBeTransformed,
                 int numberOfOutliers,
                 double maxTranslation) {

    assert(numberOfOutliers >= 0 && numberOfOutliers < destination.cols());

    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());

    std::uniform_int_distribution<> distribInt(0, destination.cols() - 1);
    std::uniform_real_distribution<> distribReal(-maxTranslation, maxTranslation);

    std::vector<bool> isOutlier(destination.cols(), false);

    for (int outlierCounter = 0; outlierCounter < numberOfOutliers; ++outlierCounter) {

        int pos = distribInt(randomNumberGenerator);

        while (isOutlier[pos]) {
            pos = distribInt(randomNumberGenerator);
        }

        Eigen::Vector3d translation1, translation2;
        for (int i = 0; i < translation1.size(); ++i) {
            translation1[i] = distribReal(randomNumberGenerator);
            translation2[i] = distribReal(randomNumberGenerator);
        }

        destination.col(pos).topLeftCorner<3, 1>() = translation1;
        toBeTransformed.col(pos).topLeftCorner<3, 1>() = translation2;

        isOutlier[pos] = true;
    }
}


TEST(testLoRANSAC, successfulEstimationWithOutliersUmeyama) {

    int dim = 3;
    int successfulIterations = 0;
    int totalIterations = 20;

    int numberOutliers = 100;
    const int numberOfPoints = 500;

    double noiseMaxValue = 0.01;

    int minSuccessIterations = 12;
    double meanErrorTreshold = 1e-2;

    double maxTranslation = 0.5;

    double maxErrorRotation = 0.01;
    double maxErrorTranslation = 0.01;

    double inliersFoundPart = 0.6;


    for (int iterations = 0; iterations < totalIterations; ++iterations) {
        gdr::ParamsRANSAC defaultRansacL2;
        defaultRansacL2.setProjectionUsage(false);

        gdr::EstimatorRobustLoRANSAC estimatorRobustLoRansacUmeyama(gdr::InlierCounter(),
                                                                    defaultRansacL2);
        gdr::SE3 transformationSE3 = gdr::SE3::getRandomSE3(maxTranslation);


        Eigen::MatrixXd noise(dim + 1, numberOfPoints);
        noise.block(0, 0, dim, numberOfPoints) = getRandomMatrixLowRowOnes(numberOfPoints,
                                                                           noiseMaxValue,
                                                                           2);

        Eigen::Matrix4Xd toBeTransformedPoints = getRandomMatrixLowRowOnes(numberOfPoints,
                                                                           2 * maxTranslation);

        Eigen::Matrix4Xd destinationPoints = transformationSE3.getSE3().matrix()
                                             * (toBeTransformedPoints + noise);

        setOutliers(destinationPoints, toBeTransformedPoints, numberOutliers, maxTranslation);

        bool success;
        std::vector<int> inlierIndices = {};

        gdr::SE3 transformationSe3Robust = estimatorRobustLoRansacUmeyama
                .estimateRelativePose(toBeTransformedPoints,
                                      destinationPoints,
                                      gdr::CameraRGBD(),
                                      gdr::CameraRGBD(),
                                      success,
                                      inlierIndices);


        auto errors = transformationSe3Robust.getRotationTranslationErrors(transformationSE3);


        ASSERT_GE(1.0 * inlierIndices.size() / (numberOfPoints - numberOutliers), inliersFoundPart);
        ASSERT_LE(errors.first, maxErrorRotation);
        ASSERT_LE(errors.second, maxTranslation);
    }


}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}