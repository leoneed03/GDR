//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_LOSSHUBER_H
#define GDR_LOSSHUBER_H

#include "LossFunctions.h"

namespace gdr {
    template<class T>
    class LossHuber : public LossFunction<T> {
    public:
        virtual T evaluate(T value, T threshold) override {
            T valueNorm(value);
            assert(valueNorm >= T(0.0));

            if (valueNorm < threshold) {
                return valueNorm * valueNorm * T(0.5);
            } else {
                return threshold * (valueNorm - threshold * T(0.5));
            }
        }
    };
}


#endif
