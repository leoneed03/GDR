//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "Point3d.h"

#define epsilonD (std::numeric_limits<double>::epsilon())

TEST(testPoint3dRepresentation, fromEigenVector3d) {

    Eigen::Vector3d point;
    int index = 8112;
    point[0] = 11.1223;
    point[1] = -100.1;
    point[2] = 11111;

    gdr::Point3d point3dGDR(point, index);

    auto pointFromPoint3d = point3dGDR.getEigenVector3dPointXYZ();
    double diffPointsNorm = (pointFromPoint3d - point).norm();

    ASSERT_LE(diffPointsNorm, epsilonD);
    ASSERT_EQ(point3dGDR.getIndex(), index);
}


TEST(testPoint3dRepresentation, toEigenVector4d) {

    Eigen::Vector3d point;
    int index = 1;
    point[0] = 11.1223;
    point[1] = -100.1;
    point[2] = 11111;

    gdr::Point3d point3dGDR(point, index);

    auto pointFromPoint3d = point3dGDR.getEigenVector4dPointXYZ1();
    ASSERT_LE(abs(pointFromPoint3d[3] - 1.0), epsilonD);
    double diffPointsNorm = (pointFromPoint3d.block<3,1>(0,0) - point).norm();

    ASSERT_EQ(point3dGDR.getIndex(), index);
    ASSERT_LE(diffPointsNorm, epsilonD);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}