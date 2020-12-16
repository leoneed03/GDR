//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/groundTruthTransformer.h"
#include <set>

int main() {
    std::set<int> ind;
    int step = 10;
    int frames = 20;
    for (int i = 0; i < frames * step; i += step) {
        ind.insert(i);
    }
    GTT::prepareDataset("/home/leoneed/Desktop/plant_dataset", "../data", ind, "plant_20_10");
    return 0;
}


