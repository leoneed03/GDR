//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATIONAVERAGER_H
#define GDR_ROTATIONAVERAGER_H

#include <string>
#include <vector>
#include <Eigen/Eigen>

#include "parametrization/SO3.h"

namespace gdr {

    class RotationAverager {

    public:
        //TODO: depend on  Rotation Measurement and add g2o file creator as separate class
        static std::vector<SO3>
        shanonAveraging(const std::string &pathToRelativeRotations,
                        const std::string &pathOut,
                        bool printProgressToConsole = false);

    };
}

#endif
