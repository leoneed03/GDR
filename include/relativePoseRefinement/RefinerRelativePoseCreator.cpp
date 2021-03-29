//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseRefinement/RefinerRelativePoseCreator.h"
#include "relativePoseRefinement/ICPCUDA.h"

namespace gdr {

    std::unique_ptr<IRefinerRelativePose> RefinerRelativePoseCreator::getRefiner(
            const RefinerRelativePoseCreator::TypeICP &refinerType) {

        if (refinerType == TypeICP::ICPCUDA) {

        } else {
            std::cout << " only ICPCUDA version can be used right now" << std::endl;
        }

        return std::make_unique<ICPCUDA>();
    }
}