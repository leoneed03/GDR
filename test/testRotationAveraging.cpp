//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"
#include "rotationAveraging.h"
#include "quaternions.h"

TEST(testRotationAveraging, computeAbsoluteRotationsDatasetPlant_19) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientations = correspondenceGraph.performRotationAveraging();

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
        double currentAngleError = absoluteRotationsQuatFromGroundTruth[i].angularDistance(computedAbsoluteOrientations[i]);
        std::cout << i << ":\t" << currentAngleError << std::endl;
        sumErrors += currentAngleError;
        sumErrorsSquared += pow(currentAngleError, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    ASSERT_LE(meanError, 0.15);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}