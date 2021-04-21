//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/PointXYZRGBfloatUchar.h"

namespace gdr {

    PointXYZRGBfloatUchar::PointXYZRGBfloatUchar(float X1, float Y1, float Z1,
                                                 unsigned char R1,
                                                 unsigned char G1,
                                                 unsigned char B1) :
            x(X1), y(Y1), z(Z1),
            R(R1), G(G1), B(B1) {}

    float PointXYZRGBfloatUchar::getX() const {
        return x;
    }

    float PointXYZRGBfloatUchar::getY() const {
        return y;
    }

    float PointXYZRGBfloatUchar::getZ() const {
        return z;
    }

    unsigned char PointXYZRGBfloatUchar::getR() const {
        return R;
    }

    unsigned char PointXYZRGBfloatUchar::getG() const {
        return G;
    }

    unsigned char PointXYZRGBfloatUchar::getB() const {
        return B;
    }
}