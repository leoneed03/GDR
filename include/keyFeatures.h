//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_FEATURES_H
#define GDR_FEATURES_H

#include "SiftGPU.h"
#include "KeyPoint2D.h"

#include <vector>
#include <string>
#include <iostream>

#include "fileProc.h"

namespace gdr {

//
//    std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> getKeypointsDescriptorsOneImage(
//            SiftGPU *sift,
//            const std::string &pathToTheImage);
////
//    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
//    getKeypointsDescriptorsAllImages(
//            SiftGPU *sift,
//            const std::string &pathToTheDirectory
//    );

//    std::vector<std::pair<int, int>>
//    getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
//                                 const imageDescriptor &keysDescriptors2,
//                                 SiftMatchGPU *matcher);
}

#endif
