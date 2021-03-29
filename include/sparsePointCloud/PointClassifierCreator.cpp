//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "sparsePointCloud/PointClassifier.h"
#include "PointClassifierCreator.h"


namespace gdr {

    std::unique_ptr<IPointClassifier> PointClassifierCreator::getRefiner(int numberOfPoses) {

        return std::make_unique<PointClassifier>(numberOfPoses);
    }
}