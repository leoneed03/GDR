#include "../include/groundTruthTransformer.h"

int main() {
    std::set<int> indicesSet;
    for (int i = 0; i < 200; i += 10) {
        indicesSet.insert(i);
    }

    ////////EXAMPLE WORKS RIGHT
    GTT::prepareDataset("/home/leoneed/Desktop/plant_dataset", "/home/leoneed/CLionProjects/GDR/test/data", indicesSet, "plantSampled_20");
//    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/Desktop/coke_dataset/groundtruth.txt",
//                                               "/home/leoneed/Desktop/coke_dataset/rgb",
//                                               "/home/leoneed/Desktop/coke_dataset/depth",
//                                               "/home/leoneed/CLionProjects/GDR/test/data/coke",
//                                               "/home/leoneed/Desktop/coke_dataset/rgb.txt",
//                                               indicesSet);
//    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test/data/coke/groundtruth_new.txt", "/home/leoneed/CLionProjects/GDR/test/data/coke/groundtruthR.txt");


//    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/groundtruth.txt",
//                                               "/home/leoneed/Desktop/360dataset/rgb",
//                                               "/home/leoneed/Desktop/360dataset/depth",
//                                               "/home/leoneed/CLionProjects/GDR/test/data/360_20",
//                                               indicesSet);
//    GTT::makeRotationsRelativeAndExtractImages("/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/groundtruth.txt",
//                                               "/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/rgb_original",
//                                               "/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/depth_original",
//                                               "/home/leoneed/CLionProjects/GDR/test/data/extracted",
//                                               indicesSet);

//    GTT::makeRotationsRelative("/home/leoneed/CLionProjects/GDR/test/data/rgbdoffice/groundtruth.txt", "relativeGroundTruth.txt");
    return 0;
}