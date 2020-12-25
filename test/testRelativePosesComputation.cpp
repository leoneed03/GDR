//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"

TEST(testRelativePosesComputation, getPairwiseTransformations) {


    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth",
                                                 525.0,
                                                 319.5, 525.0, 239.5);
    correspondenceGraph.computeRelativePoses();
    std::string pathToGroundTruth = "../../data/plantFirst_20_2/groundtruth_new.txt";
    std::string estimatedPairWise = correspondenceGraph.relativePose;
    ASSERT_TRUE(correspondenceGraph.verticesOfCorrespondence.size() == 19);
    std::pair<gdr::errorStats, gdr::errorStats> errorsStatsTR = gdr::getErrorStatsTranslationRotationFromGroundTruthAndEstimatedPairWise(
            pathToGroundTruth, estimatedPairWise);
    std::cout << "=====================================" << std::endl;
    std::cout << "translation stats: " << errorsStatsTR.first.meanError << " with standart deviation "
              << errorsStatsTR.first.standartDeviation << std::endl;
    std::cout << "rotation    stats: " << errorsStatsTR.second.meanError << " with standart deviation "
              << errorsStatsTR.second.standartDeviation << std::endl;

    ASSERT_TRUE(errorsStatsTR.first.meanError < 0.2);
    ASSERT_TRUE(errorsStatsTR.second.meanError < 0.01);
}

int main(int argc, char *argv[]) {

    std::string s = "hi.txt";
    std::ofstream outp(s);
    outp << "HELLP" << std::endl;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

