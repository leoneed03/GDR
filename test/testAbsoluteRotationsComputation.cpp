//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"



TEST(testAbsoluteRotationsComputation, onlyShonanNoBA) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
//    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuatFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteRotationsQuatFromGroundTruth.push_back(posesInfo[i].orientationQuat);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuatFromGroundTruth,
                                                        absoluteRotationsQuatFromGroundTruth[0].inverse().normalized());


    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(computedAbsoluteOrientationsNoRobust[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "shonan rotation averaging result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    ASSERT_LE(meanError, 0.1);
}


TEST(testAbsoluteRotationsComputation, robustNoBA) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<Eigen::Quaterniond> absoluteRotationsQuatFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteRotationsQuatFromGroundTruth.push_back(posesInfo[i].orientationQuat);
    }

    gdr::rotationOperations::applyRotationToAllFromLeft(absoluteRotationsQuatFromGroundTruth,
                                                        absoluteRotationsQuatFromGroundTruth[0].inverse().normalized());


    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(computedAbsoluteOrientationsRobust[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }

    assert(computedAbsoluteOrientationsNoRobust.size() == computedAbsoluteOrientationsRobust.size());

    double sumErrorsNoRobust = 0;
    for (int i = 0; i < computedAbsoluteOrientationsNoRobust.size(); ++i) {
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(computedAbsoluteOrientationsNoRobust[i]);
        sumErrorsNoRobust += currentAngleError;
    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    ASSERT_LE(sumErrors, sumErrorsNoRobust + 0.01);
    std::cout << "robust rotation optimization result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    ASSERT_LE(meanError, 0.07);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

