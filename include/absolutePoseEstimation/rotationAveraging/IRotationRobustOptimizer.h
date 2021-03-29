//
// Created by leo on 3/29/21.
//

#ifndef GDR_IROTATIONROBUSTOPTIMIZER_H
#define GDR_IROTATIONROBUSTOPTIMIZER_H

#include "parametrization/SO3.h"

namespace gdr {
    class IRotationRobustOptimizer {
    public:

        virtual std::vector<SO3> getOptimizedOrientation(int indexFixedPose = 0) const = 0;

        virtual ~IRotationRobustOptimizer() = default;
    };
}

#endif
