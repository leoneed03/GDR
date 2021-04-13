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


TEST(testDatasetReaderBundleFusion, copyroom) {


    gdr::ReaderBundleFusion readerBundleFusion("/home/leoneed/Desktop/datasets/bundleFusion/copyroom0/copyroom",
                            "/home/leoneed/CLionProjects/GDR/data/copyroom_again_multiplied_by_5_sampled_20_1");
    readerBundleFusion.save(20);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
