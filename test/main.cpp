//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "../include/CorrespondenceGraph.h"

using namespace std;

int main(int argc, char **argv) {

    CorrespondenceGraph correspondenceGraph("../data/plantSampled_20/rgb", "../data/plantSampled_20/depth", 525.0, 319.5, 525.0, 239.5);
    return 0;
}
