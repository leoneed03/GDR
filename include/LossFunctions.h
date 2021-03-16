//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_LOSSFUNCTIONS_H
#define GDR_LOSSFUNCTIONS_H

namespace gdr {
   /*
    * Huber Loss Function from
    * https://en.wikipedia.org/wiki/Huber_loss
    *
    * L_delta(a) =  0.5 * a^2,                      |a| <  d
    *               delta * (|a| - 0.5 * delta),    |a| >= d
    *
    */
    class LossFunction {
    public:
        template<class T>
        static T evaluate(T value, T threshold) {
//            T valueNorm = (value > T(0.0)) ? value : -value;
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
