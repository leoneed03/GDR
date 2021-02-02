//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"

TEST(testRelativePosesComputation, getPairwiseTransformations) {

//517.3	516.5	318.6	255.3
    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
//    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
//                                                 525.0,
//                                                 319.5, 525.0, 239.5);
//    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
//    517.3,	516.5,	318.6,	255.3);
    correspondenceGraph.computeRelativePoses();
    std::string pathToGroundTruth = "../../data/plantDataset_19_3/groundtruth_new.txt";
    std::string estimatedPairWise = correspondenceGraph.relativePose;
    ASSERT_TRUE(correspondenceGraph.verticesOfCorrespondence.size() == 19);
    std::pair<gdr::errorStats, gdr::errorStats> errorsStatsTR = gdr::getErrorStatsTranslationRotationFromGroundTruthAndEstimatedPairWise(
            pathToGroundTruth, estimatedPairWise);
    std::cout << "=====================================" << std::endl;
    std::cout << "translation stats: " << errorsStatsTR.first.meanError << " with standard deviation "
              << errorsStatsTR.first.standartDeviation << std::endl;
    std::cout << "rotation    stats: " << errorsStatsTR.second.meanError << " with standard deviation "
              << errorsStatsTR.second.standartDeviation << std::endl;

    correspondenceGraph.printConnectionsRelative(std::cout, 15);

    ASSERT_LE(errorsStatsTR.first.meanError, 0.15);
    ASSERT_LE(errorsStatsTR.second.meanError, 0.25);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

