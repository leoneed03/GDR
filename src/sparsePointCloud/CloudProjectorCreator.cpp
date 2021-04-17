//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "sparsePointCloud/CloudProjectorStl.h"
#include "sparsePointCloud/CloudProjectorCreator.h"

namespace gdr {

    std::unique_ptr<CloudProjector>
    gdr::CloudProjectorCreator::getCloudProjector() {

        return std::make_unique<CloudProjectorStl>();
    }

}