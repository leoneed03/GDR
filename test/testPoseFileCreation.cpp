//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"

int countNumberOfLines(const std::string &relPosesFile) {
    int numberOfLines = 0;
    std::ifstream relPoses(relPosesFile);
    std::string currentString;
    while (std::getline(relPoses, currentString)) {
        ++numberOfLines;
    }
    return numberOfLines;
}

TEST(testCorrespondenceGraph, relativePoseFileCreated) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 525.0,
                                                 319.5, 525.0, 239.5);
    std::cout << "compute poses" << std::endl;
    correspondenceGraph.computeRelativePoses();
    int numberOfLines = countNumberOfLines(correspondenceGraph.relativePose);
    int defaultNumberOfLines = 19;

    ASSERT_GE(numberOfLines, correspondenceGraph.verticesOfCorrespondence.size());
    ASSERT_GE(numberOfLines, defaultNumberOfLines);
}

TEST(testDepthTxtFile, fileCreated) {
    gdr::GTT::createDepthTxtFile("../../data/plantDataset_19_3/depth", "../../data/plantDataset_19_3/depth.txt");
}


TEST(testGroundTrurhFileCreation, isFileCreated) {

    std::set<int> indices;
    for (int i = 0; i < 20; ++i) {
        indices.insert(3 * i);
    }
//    gdr::GTT::prepareDataset("/home/leoneed/Desktop/plant_dataset", "/home/leoneed/testGDR1/GDR/data/temp", indices, "subset");
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}