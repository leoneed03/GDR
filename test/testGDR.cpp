//#include <ICPOdometry.h>
#include <SiftGPU.h>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/CorrespondenceGraph.h"

//#include <pangolin/image/image_io.h>
//#include <ICPOdometry.h>

TEST(testCorrespondanceGraph, fullTest)
{
    CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth", 525.0, 319.5, 525.0, 239.5);
    ASSERT_EQ(1, 1) << "1 is not equal 0";
}
void testCG() {

//    ICPOdometry icpOdom(640, 480, 319.5, 239.5, 528, 528);
    std::cout << "hello";
    CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth", 525.0, 319.5, 525.0, 239.5);
}
int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}