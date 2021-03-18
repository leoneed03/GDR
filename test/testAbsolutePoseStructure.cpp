//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <ostream>

#include "parametrization/Rotation3d.h"

struct p {
private:
    int x = -1;
    int y = -2;
public:
    p(int x1, int y1): x(x1), y(y1) {}
    p() {}
    int getX() const {
        return x;
    }
    friend std::ostream& operator<<(std::ostream& os, const p& pose);
};

std::ostream& operator<<(std::ostream& os, const p& pose) {
    os << pose.x << ' ' << pose.y;
}


TEST(testAbsolutePoseStruct, constructor) {
    p pose1(2, 3);
    p pose2(pose1);
    p pose3 = pose1;
    p pose4;
    std::cout << "POSE 1: " << pose1 << std::endl;
    std::cout << "POSE 2: " << pose2 << std::endl;
    std::cout << "POSE 3: " << pose3 << std::endl;
    std::cout << "POSE 4: " << pose4 << std::endl;

}

void print(const Eigen::Quaterniond& q) {
    std::cout << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
}

TEST(testRotation3d, constructor) {
    gdr::Rotation3d rot;
    std::cout << rot << std::endl;
    double epsilon3 = 3 * std::numeric_limits<double>::epsilon();

    Eigen::Quaterniond quat(1,2,3,4);
    quat.normalize();
    print(quat);
    gdr::Rotation3d rot3d(quat);
    std::cout << gdr::Rotation3d(quat) << std::endl;

    ASSERT_LE(rot3d.getUnitQuaternion().angularDistance(quat), epsilon3);
    auto rot3dOperatorEq = rot3d;
    ASSERT_LE(rot3dOperatorEq.getUnitQuaternion().angularDistance(quat), epsilon3);
    auto rot3dCopyConstructor(rot3d);
    ASSERT_LE(rot3dCopyConstructor.getUnitQuaternion().angularDistance(quat), epsilon3);
    gdr::Rotation3d rot3dFromSO3d(rot3d.getRotationSophus());
    ASSERT_LE(rot3dFromSO3d.getUnitQuaternion().angularDistance(quat), epsilon3);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
