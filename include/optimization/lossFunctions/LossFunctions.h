//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_LOSSFUNCTIONS_H
#define GDR_LOSSFUNCTIONS_H

namespace gdr {

    /**
     * Huber Loss Function from
     * https://en.wikipedia.org/wiki/Huber_loss
     *
     * L_delta(a) =  0.5 * a^2,                      |a| <  d
     *               delta * (|a| - 0.5 * delta),    |a| >= d
     */
    class LossFunctionHuber {
    public:
        template<class T>
        static T evaluate(T value, T threshold) {
            T valueNorm(value);
            assert(valueNorm >= T(0.0));

            if (valueNorm < threshold) {
                return valueNorm * valueNorm * T(0.5);
            } else {
                return threshold * (valueNorm - threshold * T(0.5));
            }
        }
    };

    class LossFunctionTukey {
    public:
        template<class T>
        static T evaluate(T value, T threshold) {
            T valueNorm(value);
            assert(valueNorm >= T(0.0));

            T c2_6 = threshold * threshold / T(6.0);
            T ri_c2 = (valueNorm / threshold) * (valueNorm / threshold);
            T one_ri_c2_3 = (T(1) - ri_c2) * (T(1) - ri_c2) * (T(1) - ri_c2);

            if (valueNorm < threshold) {
                return c2_6 * (T(1) - one_ri_c2_3);
            } else {
                return c2_6;
            }
        }
    };
}

#endif
