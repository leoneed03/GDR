#include <SiftGPU.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer.hpp>
#include <dirent.h>

//#include <experimental/filesystem>

#include <GL/gl.h>

std::vector<std::string> readRgbData(std::string pathToRGB) {
    DIR *pDIR;
    struct dirent *entry;
    std::vector<std::string> RgbImages;
    std::cout << "start reading" << std::endl;
    if ((pDIR = opendir(pathToRGB.data())) != nullptr) {
        int imageCounter = 0;
        while ((entry = readdir(pDIR)) != nullptr) {
            std::string absolutePathToRgbImage = pathToRGB + "/" + entry->d_name;
            std::cout << ++imageCounter << ": " << absolutePathToRgbImage << "\n";
            RgbImages.emplace_back(absolutePathToRgbImage);
        }
        closedir(pDIR);
    } else {
        std::cout << "Unable to open" << std::endl;
    }
    return RgbImages;
}


std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> getKeypointsDescriptorsOneImage(
        SiftGPU& sift,
        std::string pathToTheImage) {

    sift.RunSIFT(pathToTheImage.data());
    int num1 = sift.GetFeatureNum();
    std::vector<float> descriptors1(128 * num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);
    sift.GetFeatureVector(&keys1[0], &descriptors1[0]);
    std::cout << num1 << " -- totally" << std::endl;
    return {keys1, descriptors1};
}

std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> getKeypointsDescriptorsAllImages(
        SiftGPU& sift,
        std::string pathToTheDirectory) {
    std::vector<std::string> pathsToAllImages = readRgbData(pathToTheDirectory);
    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keypointsAndDescriptorsAllImages;
    keypointsAndDescriptorsAllImages.reserve(pathsToAllImages.size());
    for (const auto& pathToTheImage: pathsToAllImages) {
        keypointsAndDescriptorsAllImages.emplace_back(getKeypointsDescriptorsOneImage(sift, pathToTheImage));
    }
    return keypointsAndDescriptorsAllImages;
}
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
    /*
    sift.RunSIFT("/home/leoneed/Desktop/dataset/rgbd2/office1.png");
    cout << "siftgpu::runsift() cost time=" << timer.elapsed() << endl;

    int num1 = sift.GetFeatureNum();
    vector<float> descriptors1(128 * num1);
    vector<SiftGPU::SiftKeypoint> keys1(num1);
    cout << "Feature number=" << num1 << endl;

    timer.restart();
    sift.GetFeatureVector(&keys1[0], &descriptors1[0]);*/
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

    SiftMatchGPU *matcher = new SiftMatchGPU(4096);
    matcher->VerifyContextGL(); //must call once
    auto keysDescriptorsAll = getKeypointsDescriptorsAllImages(sift, "../data/rgbdoffice/rgb");

    {
        auto keysDescriptors1 = keysDescriptorsAll[11];
        auto keysDesctiptors2 = keysDescriptorsAll[35];

        auto descriptors11 = keysDescriptors1.second;
        auto descriptors22 = keysDescriptors2.second;

        auto keys11 = keysDescriptors1.first;
        auto keys22 = keysDescriptors2.first;

        auto num11 = keys11.size();
        auto num22 = keys22.size();

        matcher->SetDescriptors(0, num11, &descriptors11[0]); //image 1
        matcher->SetDescriptors(1, num22, &descriptors22[0]); //image 2


        int (*match_buf)[2] = new int[num11][2];
        //use the default thresholds. Check the declaration in SiftGPU.h
        int num_match = matcher->GetSiftMatch(num11, match_buf);
        std::cout << num_match << " sift matches were found;\n";

        for (int i = 0; i < num_match; ++i) {
            std::cout << i << " -> keypoint on the 1st image " << match_buf[i][0] << " keypoint on the 2nd image "
                      << match_buf[i][1] << std::endl;
        }

        delete[] match_buf;
    }

    {
//        auto keysDescriptorsAll = getKeypointsDescriptorsAllImages(sift, "../data/rgbdoffice/rgb");
        auto keysDescriptors1 = keysDescriptorsAll[60];
        auto keysDesctiptors2 = keysDescriptorsAll[65];

        auto descriptors11 = keysDescriptors1.second;
        auto descriptors22 = keysDescriptors2.second;

        auto keys11 = keysDescriptors1.first;
        auto keys22 = keysDescriptors2.first;

        auto num11 = keys11.size();
        auto num22 = keys22.size();

        matcher->SetDescriptors(0, num11, &descriptors11[0]); //image 1
        matcher->SetDescriptors(1, num22, &descriptors22[0]); //image 2


        int (*match_buf)[2] = new int[num11][2];
        //use the default thresholds. Check the declaration in SiftGPU.h
        int num_match = matcher->GetSiftMatch(num11, match_buf);
        std::cout << num_match << " sift matches were found;\n";

        /*
        for (int i = 0; i < num_match; ++i) {
            std::cout << i << " -> keypoint on the 1st image " << match_buf[i][0] << " keypoint on the 2nd image "
                      << match_buf[i][1] << std::endl;
        }*/

        delete[] match_buf;
    }
    delete matcher;

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
