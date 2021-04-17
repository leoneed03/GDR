//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESTIMATORRELATIVEPOSEROBUSTCREATOR_H
#define GDR_ESTIMATORRELATIVEPOSEROBUSTCREATOR_H

#include <memory>

#include "relativePoseEstimators/EstimatorRelativePoseRobust.h"
#include "relativePoseEstimators/InlierCounter.h"

namespace gdr {

    class EstimatorRelativePoseRobustCreator {

    public:
        EstimatorRelativePoseRobustCreator() = delete;

        enum class EstimatorMinimal {
            UMEYAMA
        };
        enum class EstimatorScalable {
            UMEYAMA
        };

        static std::unique_ptr<EstimatorRelativePoseRobust> getEstimator(const InlierCounter &inlierCounterToSet,
                                                                         const ParamsRANSAC &paramsRansac,
                                                                         const EstimatorMinimal &estimatorMinimal,
                                                                         const EstimatorScalable &estimatorScalable);
    };
}

#endif
