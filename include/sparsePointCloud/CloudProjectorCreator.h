//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CLOUDPROJECTORCREATOR_H
#define GDR_CLOUDPROJECTORCREATOR_H

#include <memory>

#include "sparsePointCloud/CloudProjector.h"
#include "sparsePointCloud/ProjectableInfo.h"

namespace gdr {

    class CloudProjectorCreator {

    public:

        CloudProjectorCreator() = delete;

        static std::unique_ptr<CloudProjector> getCloudProjector();
    };
}

#endif
