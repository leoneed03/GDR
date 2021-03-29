//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <cassert>
#include "keyPoints/KeyPoint2DAndDepth.h"

namespace gdr {

    KeyPoint2DAndDepth::KeyPoint2DAndDepth(double newX,
                                           double newY,
                                           double newScale,
                                           double newOrientation) :
            x(newX),
            y(newY),
            scale(newScale),
            orientation(newOrientation) {}

    KeyPoint2DAndDepth::KeyPoint2DAndDepth(double newX, double newY) :
            x(newX), y(newY) {}

    double KeyPoint2DAndDepth::getX() const {
        return x;
    }

    double KeyPoint2DAndDepth::getY() const {
        return y;
    }

    double KeyPoint2DAndDepth::getScale() const {
        return scale;
    }

    double KeyPoint2DAndDepth::getOrientation() const {
        return orientation;
    }

    bool KeyPoint2DAndDepth::scaleOrientationAreUsable() const {
        return (scale > defValue) && (orientation > defValue);
    }

    KeyPoint2DAndDepth::KeyPoint2DAndDepth() :
            x(getDefValue()),
            y(getDefValue()) {}

    bool KeyPoint2DAndDepth::isUsable() const {
        return (x > getDefValue()) && (y > getDefValue());
    }

    void KeyPoint2DAndDepth::setDepth(double newDepth) {
        depth = newDepth;
    }

    double KeyPoint2DAndDepth::getDepth() const {
        assert(isDepthUsable() && "depth is not initialized");
        return depth;
    }

    bool KeyPoint2DAndDepth::isDepthUsable() const {
        return (depth > getDefValue());
    }

    double KeyPoint2DAndDepth::getDefValue() {
        return defValue;
    }
}