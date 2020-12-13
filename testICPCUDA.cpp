#include <ICPOdometry.h>
#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "include/CorrespondenceGraph.h"

#include <pangolin/image/image_io.h>
#include <ICPOdometry.h>

void testCG() {

    ICPOdometry icpOdom(640, 480, 319.5, 239.5, 528, 528);
    std::cout << "hello";
    CorrespondenceGraph correspondenceGraph("../data/plantSampled_20/rgb", "../data/plantSampled_20/depth", 525.0, 319.5, 525.0, 239.5);

}
int main() {
    std::vector<std::vector<int>> pixels;
    for (int i = 0; i < 480; ++i) {
        pixels.push_back({i, i, 60000});
    }
    pangolin::TypedImage img = LoadImage(pixels, 640, 480, 640);

    return 0;
}