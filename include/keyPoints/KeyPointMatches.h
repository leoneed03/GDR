//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_KEYPOINTMATCHES_H
#define GDR_KEYPOINTMATCHES_H

#include "keyPoints/KeyPointInfo.h"

#include <vector>

namespace gdr {

    using keyPointImageAndLocalPointIndexAndKeyPointInfo = std::pair<std::pair<int, int>, KeyPointInfo>;

    using KeyPointMatches =
    std::vector<std::pair<keyPointImageAndLocalPointIndexAndKeyPointInfo,
            keyPointImageAndLocalPointIndexAndKeyPointInfo>>;
}


#endif
