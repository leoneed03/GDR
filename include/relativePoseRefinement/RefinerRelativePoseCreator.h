//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_REFINERRELATIVEPOSECREATOR_H
#define GDR_REFINERRELATIVEPOSECREATOR_H

#include <memory>

#include "RefinerRelativePoseCreator.h"
#include "relativePoseRefinement/IRefinerRelativePose.h"

namespace gdr {

    class RefinerRelativePoseCreator {

    public:

        RefinerRelativePoseCreator() = delete;

        enum class TypeICP {
            ICPCUDA
        };

        static std::unique_ptr<IRefinerRelativePose> getRefiner(const TypeICP &refinerType);
    };
}

#endif
