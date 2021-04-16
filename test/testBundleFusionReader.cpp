//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "readerDataset/readerBundleFusion/ReaderBundleFusion.h"
#include "readerDataset/readerTUM/ImagesAssociator.h"


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <chrono>
#include "sophus/se3.hpp"
#include "cmath"
#include "random"

#include "readerDataset/readerTUM/ClosestMatchFinder.h"
#include "parametrization/PoseFullInfo.h"
#include "readerDataset/readerTUM/ReaderTum.h"

TEST(testDatasetReaderBundleFusion, copyroom) {

    auto posesGT = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation("/home/leoneed/Desktop/datasets/freiburg/rgbd_dataset_freiburg1_plant/groundtruth.txt");

    boost::filesystem::path depth("/home/leoneed/CLionProjects/GDR/data/plant_sampled_19_3/depth");

    std::vector<double> timestamps;

    for (boost::filesystem::directory_iterator end_dir_it, it(depth); it != end_dir_it; ++it)  {
        std::string name = it->path().filename().string();
        timestamps.emplace_back(std::stod(name.substr(0, name.length() - 4)));
    }
    std::sort(timestamps.begin(), timestamps.end());
    auto posesAssoc = gdr::ImageAssociator::getGroundtruthForGivenTimestamps(timestamps, posesGT, 0.02);
    std::ofstream gt("/home/leoneed/CLionProjects/GDR/data/plant_sampled_19_3/groundtruth.txt");
    for (const auto& pose: posesAssoc) {
        gt << pose << std::endl;
    }

//    gdr::ReaderBundleFusion readerBundleFusion("/home/leoneed/Desktop/datasets/bundleFusion/copyroom0/copyroom",
//                            "/home/leoneed/CLionProjects/GDR/data/copyroom_again_multiplied_by_5_sampled_20_1");
//    readerBundleFusion.save(20);
}

int main(int argc, char *argv[]) {

//    gdr::ReaderTUM::createRgbOrDepthTxt("/home/leoneed/CLionProjects/GDR/data/desk1_sampled_98_6/depth",
//                                        "/home/leoneed/CLionProjects/GDR/data/desk1_sampled_98_6");
//    gdr::ReaderTUM::createRgbOrDepthTxt("/home/leoneed/CLionProjects/GDR/data/desk1_sampled_98_6/rgb",
//                                        "/home/leoneed/CLionProjects/GDR/data/desk1_sampled_98_6");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
