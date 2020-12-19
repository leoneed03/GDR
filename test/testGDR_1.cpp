//#include <ICPOdometry.h>
#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/CorrespondenceGraph.h"

//#include <pangolin/image/image_io.h>
//#include <ICPOdometry.h>

void testCG() {

//    ICPOdometry icpOdom(640, 480, 319.5, 239.5, 528, 528);
    std::cout << "hello";
    CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth", 525.0, 319.5, 525.0, 239.5);

}
int main() {
    testCG();
//    std::vector<std::vector<int>> pixels;
//    for (int i = 0; i < 200; ++i) {
//        pixels.push_back({i, i, i});
//    }
//
//    std::cout << "start creating" << std::endl;
//    pangolin::TypedImage img = LoadImageT(pixels, 640, 480, 640);
//
//    std::cout << "end loading" << std::endl;
//
//
//    pangolin::PixelFormat raw_fmt = pangolin::PixelFormatFromString("GRAY16LE");
//
//    std::cout << "start saving" << std::endl;
//    pangolin::SaveImage(img, "pango.png");
    return 0;
}