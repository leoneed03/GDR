//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONAVERAGING_H
#define GDR_ROTATIONAVERAGING_H

#include <string>
#include <Eigen/Eigen>

namespace gdr {

    struct rotationAverager {

        static std::vector<Eigen::Quaterniond> shanonAveraging(const std::string &pathToRelativeRotations, const std::string &pathOut);
    };
}

#endif
