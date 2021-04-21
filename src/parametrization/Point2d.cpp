//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/Point2d.h"

namespace gdr {
    Point2d::Point2d(double xToSet, double yToSet) :
            x(xToSet), y(yToSet) {}

    double Point2d::getX() const {
        return x;
    }

    double Point2d::getY() const {
        return y;
    }
}