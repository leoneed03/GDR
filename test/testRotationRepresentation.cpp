//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "Rotation3d.h"

#define epsilonD3 (10 * std::numeric_limits<double>::epsilon())

TEST(testRotationRepresentation, fromQuaternion) {

    Eigen::Quaterniond randomQuaternion = gdr::Rotation3d::getRandomUnitQuaternion();
    gdr::Rotation3d rotationRandomGDR(randomQuaternion);

    Eigen::Quaterniond quaternionFromRotationGDR = rotationRandomGDR.getUnitQuaternion();

    double quaternionDifference = quaternionFromRotationGDR.angularDistance(randomQuaternion);

    ASSERT_LE(quaternionDifference, epsilonD3);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}