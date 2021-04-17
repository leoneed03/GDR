//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "sparsePointCloud/PointClassifierStl.h"
#include "sparsePointCloud/PointClassifierCreator.h"

namespace gdr {

    std::unique_ptr<PointClassifier> PointClassifierCreator::getPointClassifier() {

        return std::make_unique<PointClassifierStl>();
    }
}