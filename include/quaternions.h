//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_QUATERNIONS_H
#define GDR_QUATERNIONS_H

#include "util.h"

namespace gdr {

    std::vector<Eigen::Matrix3d> getRotationsFromQuaternionVector(const std::vector<std::vector<double>> &quats);
}

#endif
