#include "../include/groundTruthTransformer.h"

int main() {
    std::set<int> indicesSet;
    for (int i = 0; i < 200; i += 10) {
        indicesSet.insert(i);
    }

    ////////EXAMPLE WORKS RIGHT
    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/Desktop/coke_dataset/groundtruth.txt",
                                               "/home/leoneed/Desktop/coke_dataset/rgb",
                                               "/home/leoneed/Desktop/coke_dataset/depth",
                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/coke",
                                               "/home/leoneed/Desktop/coke_dataset/rgb.txt",
                                               indicesSet);
    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/coke/groundtruth_new.txt", "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/coke/groundtruthR.txt");


//    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt",
//                                               "/home/leoneed/Desktop/360dataset/rgb",
//                                               "/home/leoneed/Desktop/360dataset/depth",
//                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/360_20",
//                                               indicesSet);
//    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt",
//                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/rgb_original",
//                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/depth_original",
//                                               "/home/leoneed/CLionProjects/GDR/test_siftgpu/data/extracted",
//                                               indicesSet);

//    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test_siftgpu/data/rgbdoffice/groundtruth.txt", "relativeGroundTruth.txt");
    return 0;
}