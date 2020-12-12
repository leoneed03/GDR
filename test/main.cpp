#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/CorrespondenceGraph.h"

#include <pangolin/image/image_io.h>
#include <ICPOdometry.h>

using namespace std;

void opencv() {
    typedef cv::Mat Image;
    Image imgA = cv::imread("/home/leoneed/Desktop/testsiftcuda/test/data/640.jpg",
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
    sift.RunSIFT("/home/leoneed/Desktop/testsiftcuda/test/data/640.jpg");
//    sift.RunSIFT("/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/rgb/1305031102.175304.png");

    int num = sift.GetFeatureNum();
    std::vector<float> descriptors(128 * num);
    std::vector<SiftGPU::SiftKeypoint> keys(num);

    sift.GetFeatureVector(&keys[0], &descriptors[0]);
    cv::Mat image = cv::imread("/home/leoneed/Desktop/testsiftcuda/test/data/640.jpg");

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

void testTextReading() {
    std::vector<std::vector<double>> quaternions = parseAbsoluteRotationsFile("absoluteRotations.txt");
    std::cout << "number " << quaternions.size() << std::endl;
    for (const auto& e: quaternions) {
        for (const auto& a: e) {
            std::cout << std::setw(15) << a;
        }
        std::cout << std::endl;
    }
    auto res = getRotationsFromQuaternionVector(quaternions);
    int count0 = 0;
    auto absoluteRotationTranslation = getSomeMatrix(4, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            absoluteRotationTranslation.col(i)[j] = 0;
        }
    }
    absoluteRotationTranslation.row(3)[3] = 1;
    for (const auto& e: res) {
        std:: cout << count0 << "\n";
        count0++;
        std::cout << e;
        std::cout << "\n===============================================\n";
    }
    absoluteRotationTranslation.block<3,3>(0,0) = res[res.size() - 1];

    std::cout << "\n===============================================\n";

    std::cout << absoluteRotationTranslation;
    std::cout << "\n===============================================\n";

}

int main(int argc, char **argv) {

    pangolin::ManagedImage<unsigned short> firstData(640, 480);
    CorrespondenceGraph correspondenceGraph("../data/plantSampled_20/rgb", "../data/plantSampled_20/depth", 525.0, 319.5, 525.0, 239.5);
    return 0;
}
