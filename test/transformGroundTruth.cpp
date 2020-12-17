//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/groundTruthTransformer.h"
#include <set>

int main() {
    std::set<int> indicesSet;
    for (int i = 3; i < 20 * 3; i += 3) {
        indicesSet.insert(i);
    }

    ////////EXAMPLE WORKS RIGHT
    GTT::prepareDataset("/home/leoneed/Desktop/plant_dataset", "/home/leoneed/Desktop/GDR/data", indicesSet,
                        "plantFirst_20_2");
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