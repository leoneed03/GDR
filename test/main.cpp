//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//



#include "../include/CorrespondenceGraph.h"

using namespace std;

int main(int argc, char **argv) {

    CorrespondenceGraph correspondenceGraph("../data/plant_20_10/rgb", "../data/plant_20_10/depth", 525.0, 319.5, 525.0, 239.5);
    return 0;
}
