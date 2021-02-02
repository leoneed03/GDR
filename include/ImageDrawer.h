//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//



#ifndef GDR_IMAGEDRAWER_H
#define GDR_IMAGEDRAWER_H

#include "KeyPointInfo.h"

#include <string>

namespace gdr {

    struct ImageDrawer {

        static int
        showKeyPointOnImage(const std::string &pathToRGBImage,
                            const KeyPointInfo &keyPointInfo,
                            int pointIndex,
                            std::string pathToSave = "",
                            std::string nameToSave = "");
    };
}

#endif
