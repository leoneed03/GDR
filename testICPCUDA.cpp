//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <SiftGPU.h>
#include <iostream>
#include "include/CorrespondenceGraph.h"

void testCG() {
    std::cout << "hello";
    CorrespondenceGraph correspondenceGraph("../data/plantFirst_20_2/rgb", "../data/plantFirst_20_2/depth", 525.0,
                                            319.5, 525.0, 239.5);
    std::cout << "success" << std::endl;

}

int main() {
    testCG();
    return 0;
}