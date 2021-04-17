//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINTCLASSIFIERCREATOR_H
#define GDR_POINTCLASSIFIERCREATOR_H

#include <memory>

#include "sparsePointCloud/PointClassifier.h"

namespace gdr {

    class PointClassifierCreator {

    public:
        PointClassifierCreator() = delete;

        static std::unique_ptr<PointClassifier> getPointClassifier();
    };
}

#endif
