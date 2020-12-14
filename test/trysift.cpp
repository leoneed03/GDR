//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iomanip>

int trysift() {

    SiftGPU sift;
    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[4] = {"-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);

    int support = sift.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        return 1;
    }
    sift.RunSIFT("../data/640.jpg");

    int num = sift.GetFeatureNum();
    std::vector<float> descriptors(128 * num);
    std::vector<SiftGPU::SiftKeypoint> keys(num);

    sift.GetFeatureVector(&keys[0], &descriptors[0]);
    cv::Mat image = cv::imread("../data/640.jpg");
    cv::imshow("Image", image);
    cv::waitKey(0);

    for (int x = 0; x < image.cols; ++x) {
        for (int y = 0; y < image.rows; ++y) {
            auto &pixel = image.at<cv::Vec3b>((int) y, (int) x);
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 0;

        }
    }

    for (int i = 0; i < keys.size(); ++i) {
        auto &key = keys[i];
        std::cout << "keypoint number " << std::setw(4) << i << " (x::y) = " << std::setw(8) << key.x << "::"
                  << std::setw(8) << key.y << std::endl;
        auto &pixel = image.at<cv::Vec3b>((int) key.y, (int) key.x);
        pixel[0] = 255;
        pixel[1] = 255;
        pixel[2] = 255;

    }
    cv::imshow("OnlyKeypoints", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}

void getMatch(const std::string &path1, const std::string &path2) {
    SiftGPU sift;
    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[4] = {"-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);

    int support = sift.CreateContextGL();
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        return;
    }
    sift.RunSIFT(path1.data());
    int num = sift.GetFeatureNum();
    std::vector<float> descriptors(128 * num);
    std::vector<SiftGPU::SiftKeypoint> keys(num);

    sift.GetFeatureVector(&keys[0], &descriptors[0]);

}

int main() {
    return trysift();
}
