//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "groundTruthTransformer.h"

TEST(testRelativePoseRead, getRelPoses) {

    std::string RelPoses = "../../data/files/pairWiseFirstPoseZero_19_less.txt";
    std::vector<gdr::relativePose> relPoses = gdr::GTT::readRelativePoses(RelPoses);
    for (const auto& pose: relPoses) {
        std::cout << pose.getIndexFromToBeTransformed() << "->" << pose.getIndexToDestination() << " ";
        Eigen::Quaterniond relQuat = pose.getRotationRelative();
        std::cout << relQuat.x() << ' ' << relQuat.y()  << ' ' << relQuat.z()  << ' ' << relQuat.w() << std::endl;
    }
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

