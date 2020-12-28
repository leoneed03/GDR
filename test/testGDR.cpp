//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "CorrespondenceGraph.h"

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

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantFirst_20_2/rgb", "../../data/plantFirst_20_2/depth",
                                                 525.0,
                                                 319.5, 525.0, 239.5);
    correspondenceGraph.computeRelativePoses();
    int numberOfLines = countNumberOfLines(correspondenceGraph.relativePose);
    int defaultNumberOfLines = 10;

    ASSERT_GE(numberOfLines, correspondenceGraph.verticesOfCorrespondence.size());
    ASSERT_GE(numberOfLines, defaultNumberOfLines);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}