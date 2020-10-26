#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer.hpp>
#include <dirent.h>

//#include <experimental/filesystem>
#include <GL/gl.h>

//#include "../include/features.h"
#include "../include/CG.h"

using namespace std;

void opencv() {
    typedef cv::Mat Image;
    Image imgA = cv::imread("/home/leoneed/Desktop/testsiftcuda/test_siftgpu/data/640.jpg",
                            cv::IMREAD_GRAYSCALE);

}

int trysift1() {

    SiftGPU sift;
    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[4] = {"-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);

    int support = sift.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        return 1;
    }
    sift.RunSIFT("/home/leoneed/Desktop/testsiftcuda/test_siftgpu/data/640.jpg");
//    sift.RunSIFT("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/rgb/1305031102.175304.png");

    int num = sift.GetFeatureNum();
    std::vector<float> descriptors(128 * num);
    std::vector<SiftGPU::SiftKeypoint> keys(num);

    sift.GetFeatureVector(&keys[0], &descriptors[0]);
    cv::Mat image = cv::imread("/home/leoneed/Desktop/testsiftcuda/test_siftgpu/data/640.jpg");

//    cv::Mat input_bgra;
//    cv::cvtColor(image, input_bgra, cv::COLOR_BGR2BGRA);
    for (int x = 0; x < image.cols; ++x) {
        for (int y = 0; y < image.rows; ++y) {
            std::cout << x << "::" << y << std::endl;
            auto &pixel = image.at<cv::Vec3b>((int) y, (int) x);
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 0;

        }
    }
    for (const auto &key: keys) {
        std::cout << key.x << "::" << key.y << std::endl;
        auto &pixel = image.at<cv::Vec3b>((int) key.y, (int) key.x);
        pixel[0] = 255;
        pixel[1] = 255;
        pixel[2] = 255;

    }
    cv::imshow("Well..", image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}

int main(int argc, char **argv) {
    /*
    cv::Mat image = imread("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/depth/1305031102.160407.png",
                           cv::IMREAD_GRAYSCALE);

    int minBrightness = INT_MAX;
    int maxBrightness = INT_MIN;

    cv::imshow("Example - Show image in window", image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    for (int j = 0; j < image.rows; j++) {
        for (int i = 0; i < image.cols; i++) {
            if (image.at<uchar>(j, i) <= minBrightness) {
                minBrightness = image.at<uchar>(j, i);
            }
            if (image.at<uchar>(j, i) >= maxBrightness) {
                maxBrightness = image.at<uchar>(j, i);
            }
        }
    }
    cv::imshow("Example - Show image after window", image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    std::cout << "Total max brightness " << maxBrightness << std::endl;
    std::cout << "min brightness " << minBrightness << std::endl;
    */
//
//    float fx = 525.0;
//    float fy = 525.0;
//    float cx = 319.5;
//    float cy = 239.5;
    CorrespondenceGraph correspondenceGraph("../data/rgbdoffice/rgb", "../data/rgbdoffice/depth", 525.0, 319.5, 525.0, 239.5);
    return 0;
}
