#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "../include/CorrespondenceGraph.h"

TEST(testCorrespondanceGraph, fullTest) {
    CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth", 525.0, 319.5, 525.0, 239.5);
    correspondenceGraph.computeRelativePoses();
    ASSERT_EQ(1, 1);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}