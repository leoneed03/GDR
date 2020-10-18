#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer.hpp>
#include <dirent.h>

//#include <experimental/filesystem>
#include <GL/gl.h>

#include "../include/features.h"

using namespace std;


int main(int argc, char **argv) {

//    readRgbData("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/rgb");
    std::vector<std::string> imagesRgb = readRgbData("../data/rgbdoffice/rgb");
    std::cout << "Totally read " << imagesRgb.size() << std::endl;


    SiftGPU sift;

    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
//    char *myargv[5] = {"-cuda", "-fo", "-1", "-v", "1"};
    sift.ParseParam(5, myargv);
    int support = sift.CreateContextGL();
    cout << "Checking" << endl;
    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        cerr << "SiftGPU is not supported!" << endl;
        return 2;
    }

    cout << "running sift" << endl;
    boost::timer timer;
    auto keysDescriptors = getKeypointsDescriptorsOneImage(sift, "/home/leoneed/Desktop/dataset/rgbd2/office1.png");
    cout << "siftgpu::getFeatureVector() cost time=" << timer.elapsed() << endl;
    auto keys1 = keysDescriptors.first;
    auto descriptors1 = keysDescriptors.second;

    //int num1 = keys1.size();
    std::cout << "totally extracted " << keys1.size() << " and " << descriptors1.size() << std::endl;
    std::cout << "First part done\n" << std::endl;

    auto keysDescriptors2 = getKeypointsDescriptorsOneImage(sift, "/home/leoneed/Desktop/dataset/rgbd2/office2.png");
    auto keys2 = keysDescriptors2.first;
    auto descriptors2 = keysDescriptors2.second;

    //int num2 = keys2.size();
    std::cout << "totally extracted " << keys1.size() << " and " << descriptors1.size() << std::endl;
    std::cout << "Second part done\n" << std::endl;

    std::unique_ptr<SiftMatchGPU> matcher(new SiftMatchGPU(4096));
    matcher->VerifyContextGL(); //must call once
    auto keysDescriptorsAll = getKeypointsDescriptorsAllImages(sift, "../data/rgbdoffice/rgb");

    int a = 2, b = 17;
    {
        std::cout << a << " match " << b << std::endl;
        auto matchingKetpoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b], matcher.get());
        std::cout << "totally matched matchingKeypoints: " << matchingKetpoints.first.size() << " and "
                  << matchingKetpoints.second.size() << std::endl;
    }

    {
        a = 17;
        b = 33;

        std::cout << a << " match " << b << std::endl;
        auto matchingKetpoints = getMatchesKeypoints(keysDescriptorsAll[a], keysDescriptorsAll[b], matcher.get());
        std::cout << "totally matched matchingKeypoints: " << matchingKetpoints.first.size() << " and "
                  << matchingKetpoints.second.size() << std::endl;
    }
    std::cout << a << " matched " << b << std::endl;
//    delete matcher;

    return 0;

    /*
    sift.RunSIFT("/home/leoneed/Desktop/dataset/rgbd2/office2.png");
    cout << "siftgpu::runsift() cost time=" << timer.elapsed() << endl;
    int num2 = sift.GetFeatureNum();
    vector<float> descriptors2(128 * num2);
    vector<SiftGPU::SiftKeypoint> keys2(num2);

    cout << "Feature number=" << num2 << endl;

    timer.restart();
    sift.GetFeatureVector(&keys2[0], &descriptors2[0]);
    cout << "siftgpu::getFeatureVector() cost time=" << timer.elapsed() << endl;

    cout << "Second part done\n" << std::endl;

//
//    cv::Mat img = cv::imread("/home/leoneed/Desktop/dataset/psquare.png", 0);
//    int width = img.cols;
//    int height = img.rows;
//    timer.restart();
//    sift.RunSIFT(width, height, img.data);
//    cout<<"siftgpu::runSIFT() cost time="<<timer.elapsed()<<endl;
//
    SiftMatchGPU *matcher = new SiftMatchGPU(4096);
    matcher->VerifyContextGL(); //must call once
    matcher->SetDescriptors(0, num1, &descriptors1[0]); //image 1
    matcher->SetDescriptors(1, num2, &descriptors2[0]); //image 2

    int (*match_buf)[2] = new int[num1][2];
    //use the default thresholds. Check the declaration in SiftGPU.h
    int num_match = matcher->GetSiftMatch(num1, match_buf);
    std::cout << num_match << " sift matches were found;\n";

    delete matcher;
    delete[] match_buf;
    std::cout << "hello!" << std::endl;
    return 0;*/
}
