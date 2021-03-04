//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>
#include "ImagesAssociator.h"



// sometimes BA results are worse than IRLS so max error is multiplied by coefficient = 1.8
TEST(testImageAssosiator, associateFilesDefaultTreshold) {

    std::set<int> indices;
    for (int i = 0; i < 1000; i += 10) {
        indices.insert(i);
    }
    std::string pathToDataset = "/home/leo/Desktop/datasets/rgbd_dataset_freiburg1_plant";
    gdr::ImageAssociator imageAssociator(pathToDataset);

    imageAssociator.AssociateImagePairs();
    imageAssociator.exportAllInfoToDirectory("../../data/plant_sampled_101_10",
                                             indices);

}


int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

