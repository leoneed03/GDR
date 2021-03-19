//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONAVERAGING_H
#define GDR_ROTATIONAVERAGING_H

#include <string>
#include <vector>
#include <Eigen/Eigen>

#include "parametrization/SO3.h"

namespace gdr {

    class rotationAverager {

    public:
        static std::vector<SO3>
        shanonAveraging(const std::string &pathToRelativeRotations, const std::string &pathOut);

    };
}

#endif
