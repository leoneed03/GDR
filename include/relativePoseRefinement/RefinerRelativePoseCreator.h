//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_REFINERRELATIVEPOSECREATOR_H
#define GDR_REFINERRELATIVEPOSECREATOR_H

#include <memory>

#include "RefinerRelativePoseCreator.h"
#include "relativePoseRefinement/RefinerRelativePose.h"

namespace gdr {

    class RefinerRelativePoseCreator {

    public:
        RefinerRelativePoseCreator() = delete;

        enum class RefinerType {
            ICPCUDA
        };

        static std::unique_ptr<RefinerRelativePose> getRefiner(const RefinerType &refinerType);
    };
}

#endif
