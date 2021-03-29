//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseEstimators/EstimatorRelativePoseRobustCreator.h"

#include "relativePoseEstimators/EstimatorRobustLoRANSAC.h"

namespace gdr {

    std::unique_ptr<IEstimatorRelativePoseRobust>
    EstimatorRelativePoseRobustCreator::getEstimator(const InlierCounter &inlierCounter,
                                                     const ParamsRANSAC &paramsRansac,
                                                     const EstimatorMinimal &estimatorMinimal,
                                                     const EstimatorScalable &estimatorScalable) {

        if (estimatorMinimal == EstimatorMinimal::UMEYAMA && estimatorScalable == EstimatorScalable::UMEYAMA) {

        } else {
            std::cout << "only umeyama is implemented at the moment" << std::endl;
        }

        return std::make_unique<EstimatorRobustLoRANSAC>(inlierCounter, paramsRansac);
    }
}