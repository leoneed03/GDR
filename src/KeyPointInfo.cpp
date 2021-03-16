//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "KeyPointInfo.h"
#include "KeyPoint2D.h"

#include <iostream>
#include <cmath>

namespace gdr {
    KeyPointInfo::KeyPointInfo(const KeyPoint2D &newKeypoint,
                               double newDepth,
                               int newObservingPoseNumber) : x(newKeypoint.getX()),
                                                             y(newKeypoint.getY()),
                                                             scale(newKeypoint.getScale()),
                                                             orientation(newKeypoint.getOrientation()),
                                                             depth(newDepth),
                                                             observingPoseNumber(newObservingPoseNumber),
                                                             initialObservingPoseNumber(newObservingPoseNumber) {}


    double KeyPointInfo::getX() const {
        return x;
    }

    double KeyPointInfo::getY() const {
        return y;
    }

    double KeyPointInfo::getScale() const {
        return scale;
    }

    double KeyPointInfo::getOrientation() const {
        return orientation;
    }

    double KeyPointInfo::getDepth() const {
        return depth;
    }

    int KeyPointInfo::getObservingPoseNumber() const {
        return observingPoseNumber;
    }
//
//    KeyPointInfo::KeyPointInfo(const KeyPointInfo &newKeypoint) :
//            x(newKeypoint.getX()),
//            y(newKeypoint.getY()),
//            scale(newKeypoint.getScale()),
//            orientation(newKeypoint.getOrientation()),
//            depth(newKeypoint.getDepth()),
//            observingPoseNumber(newKeypoint.getObservingPoseNumber()),
//            initialObservingPoseNumber(newKeypoint.getObservingPoseNumber()) {}
//
//    KeyPointInfo &KeyPointInfo::operator=(const KeyPointInfo &newKeypoint) {
//
//        std::cout << "DEBUG = " << std::endl;
//        if (this == &newKeypoint) {
//            return *this;
//        }
//
//        x = newKeypoint.getX();
//        y = newKeypoint.getY();
//        scale = newKeypoint.getScale();
//        orientation = newKeypoint.getOrientation();
//        depth = newKeypoint.getDepth();
//        observingPoseNumber = newKeypoint.getObservingPoseNumber();
//
//        return *this;
//    }

    bool KeyPointInfo::operator==(const KeyPointInfo &right) {

        if (std::abs(x - right.getX()) > epsilonD
            || std::abs(y - right.getY()) > epsilonD
            || std::abs(scale - right.getScale()) > epsilonD
            || std::abs(orientation - right.getOrientation()) > epsilonD
            || std::abs(depth - right.getDepth()) > epsilonD
            || observingPoseNumber != right.getObservingPoseNumber()) {
            return false;
        } else {
            return true;
        }
    }

    KeyPointInfo::KeyPointInfo() {
        x = getDefValue();
        y = getDefValue();
        scale = getDefValue();
        orientation = getDefValue();
        depth = getDefValue();
        observingPoseNumber = getDefValue();
        initialObservingPoseNumber = observingPoseNumber;
    }

    double KeyPointInfo::getDefValue() const {
        return defValue;
    }

    bool KeyPointInfo::isInitialized() const {
        if (std::abs(x - getDefValue()) < epsilonD
            || std::abs(y - getDefValue()) < epsilonD
            || std::abs(scale - getDefValue()) < epsilonD
            || std::abs(orientation - getDefValue()) < epsilonD
            || std::abs(depth - getDefValue()) < epsilonD) {
            return false;
        } else {
            return true;
        }
    }

    void KeyPointInfo::setObservingPoseNumber(int newObservingPoseNumber) {
        observingPoseNumber = newObservingPoseNumber;
    }

    int KeyPointInfo::getInitObservingnumber() const {
        return initialObservingPoseNumber;
    }
}
