//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>

#include "translationAveraging.h"

TEST(testTranslationAveraging, translationRecovery2Poses) {

    gdr::translationMeasurement relativeT(Eigen::Vector3d(1,-1,2), 0, 1);
    std::vector<gdr::translationMeasurement> relativeTs = {relativeT};
    std::vector<Eigen::Matrix4d> absolutePoses(2);
    absolutePoses[0].setIdentity();
    absolutePoses[1].setIdentity();
    gdr::translationAverager::recoverTranslations(relativeTs, absolutePoses);
    ASSERT_TRUE(true);
}

TEST(testTranslationAveraging, translationRecovery3Poses) {

    gdr::translationMeasurement relativeT0_1(Eigen::Vector3d(1,-1,2), 0, 1);
    gdr::translationMeasurement relativeT0_2(Eigen::Vector3d(0,10,-11), 0, 2);
    std::vector<gdr::translationMeasurement> relativeTs;
    relativeTs.push_back(relativeT0_1);
    relativeTs.push_back(relativeT0_2);
    std::vector<Eigen::Matrix4d> absolutePoses(3);
    for (auto& pose: absolutePoses) {
        pose.setIdentity();
    }
    gdr::translationAverager::recoverTranslations(relativeTs, absolutePoses);
    ASSERT_TRUE(true);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}