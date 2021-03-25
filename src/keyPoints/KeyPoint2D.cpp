//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <cassert>
#include "keyPoints/KeyPoint2D.h"

namespace gdr {

    KeyPoint2D::KeyPoint2D(double newX,
                           double newY,
                           double newScale,
                           double newOrientation) :
            x(newX),
            y(newY),
            scale(newScale),
            orientation(newOrientation) {}

    KeyPoint2D::KeyPoint2D(double newX, double newY) :
            x(newX), y(newY) {}

    double KeyPoint2D::getX() const {
        return x;
    }

    double KeyPoint2D::getY() const {
        return y;
    }

    double KeyPoint2D::getScale() const {
        return scale;
    }

    double KeyPoint2D::getOrientation() const {
        return orientation;
    }

    bool KeyPoint2D::scaleOrientationAreUsable() const {
        return (scale > defValue) && (orientation > defValue);
    }

    KeyPoint2D::KeyPoint2D() :
            x(getDefValue()),
            y(getDefValue()) {}

    bool KeyPoint2D::isUsable() const {
        return (x > getDefValue()) && (y > getDefValue());
    }

    void KeyPoint2D::setDepth(double newDepth) {
        depth = newDepth;
    }

    double KeyPoint2D::getDepth() const {
        assert(isDepthUsable() && "depth is not initialized");
        return depth;
    }

    bool KeyPoint2D::isDepthUsable() const {
        return (depth > getDefValue());
    }

    double KeyPoint2D::getDefValue() {
        return defValue;
    }
}