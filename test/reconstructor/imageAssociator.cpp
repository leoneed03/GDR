//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include "TesterReconstruction.h"
#include "readerDataset/readerTUM/ImagesAssociator.h"

int main(int argc, char* argv[]) {

    std::cout << "input args format: [path Dataset] [pathOutSampledDataset] [samplingRate], "
    << "optionaly: " << "[maxIndex]" << std::endl;
    std::cout << "your args is " << argc << std::endl;
    assert(argc == 4 || argc == 5);

    std::string pathDatasetRoot(argv[1]);
    std::string pathOutSampledDataset(argv[2]);
    int samplingRate = std::stoi(std::string(argv[3]));

    std::set<int> indicesToSample;

    int maxIndex = 6000;

    if (argc > 4) {
        maxIndex = std::stoi(std::string(argv[4]));
    }
    for (int i = 0; i < maxIndex; i += samplingRate) {
        indicesToSample.insert(i);
    }

    gdr::ImageAssociator imageAssociator(pathDatasetRoot);
    imageAssociator.associateImagePairs();
    imageAssociator.exportAllInfoToDirectory(pathOutSampledDataset, indicesToSample);

    return 0;
}