//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "LossHuber.h"

#include "cmath"

namespace gdr {
//
//    template<typename T>
//    T LossHuber::evaluate(T value, T scaleParam) {
//
//        T valueNorm = value > T(0) ? value : -value;
//
//        if (valueNorm < scaleParam) {
//            return valueNorm * valueNorm * T(0.5);
//        } else {
//            return scaleParam * (valueNorm - scaleParam * T(0.5));
//        }
//    }
}