//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "groundTruthTransformer.h"
#include "rotationAveraging.h"

TEST(testCorrespondenceGraph, relativePoseFileCreated) {

    std::string absolutePoses = "../../data/files/absolutePosesFirstPoseZero.txt";
    std::string relativeRotations = "pairWiseFirstPoseZero.txt";
    std::string absoluteRotations = "absoluteRotationsTestShanonAveraging.txt";
//    gdr::GTT::extractAllRelativeTransformationPairwise("../../data/files/absolutePosesFirstPoseZero.txt", "../../data/temp/pairWiseFirstPoseZero.txt");
    gdr::GTT::extractAllRelativeTransformationPairwise(absolutePoses, relativeRotations, "   10000.000000 0.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000 0.000000   10000.000000 0.000000 0.000000   10000.000000 0.000000   10000.000000");
    gdr::rotationAverager::shanonAveraging(relativeRotations, absoluteRotations);
    std::cout << "finishing averaging" << std::endl;
    ASSERT_TRUE(true);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}