//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/groundTruthTransformer.h"

int main() {
    std::set<int> indicesSet;
    for (int i = 0; i < 200; i += 10) {
        indicesSet.insert(i);
    }

    ////////EXAMPLE WORKS RIGHT
    GTT::prepareDataset("/home/leoneed/Desktop/plant_dataset", "/home/leoneed/CLionProjects/GDR/test/data", indicesSet,
                        "plantSampled_20");
    return 0;
}
