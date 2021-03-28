//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "keyPoints/KeyPointInfo.h"
#include "keyPoints/KeyPoint2D.h"

#include <iostream>
#include <cmath>
#include <cassert>

namespace gdr {
    KeyPointInfo::KeyPointInfo(const KeyPoint2D &keyPointToSet,
                               int observingPoseNumberToSet) : keyPoint2D(keyPointToSet),
                                                               observingPoseNumber(observingPoseNumberToSet),
                                                               initialObservingPoseNumber(observingPoseNumberToSet) {}


    double KeyPointInfo::getX() const {
        return keyPoint2D.getX();
    }

    double KeyPointInfo::getY() const {
        return keyPoint2D.getY();
    }

    double KeyPointInfo::getScale() const {
        return keyPoint2D.getScale();
    }

    double KeyPointInfo::getOrientation() const {
        return keyPoint2D.getOrientation();
    }

    double KeyPointInfo::getDepth() const {
        return keyPoint2D.getDepth();
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

        if (std::abs(getX() - right.getX()) > epsilonD
            || std::abs(getY() - right.getY()) > epsilonD
            || std::abs(getScale() - right.getScale()) > epsilonD
            || std::abs(getOrientation() - right.getOrientation()) > epsilonD
            || std::abs(getDepth() - right.getDepth()) > epsilonD
            || getObservingPoseNumber() != right.getObservingPoseNumber()) {
            return false;
        } else {
            return true;
        }
    }

    KeyPointInfo::KeyPointInfo() {
        observingPoseNumber = getDefValue();
        initialObservingPoseNumber = observingPoseNumber;
    }

    double KeyPointInfo::getDefValue() const {
        return keyPoint2D.getDefValue();
    }

    bool KeyPointInfo::isInitialized() const {
        bool xNotInit = std::abs(getX() - getDefValue()) < epsilonD;
        if (xNotInit
            || std::abs(getY() - getDefValue()) < epsilonD
            || std::abs(getScale() - getDefValue()) < epsilonD
            || std::abs(getOrientation() - getDefValue()) < epsilonD
            || std::abs(getDepth() - getDefValue()) < epsilonD
                ) {
            return false;
        } else {
            return true;
        }
    }

    void KeyPointInfo::setObservingPoseNumber(int newObservingPoseNumber) {
        observingPoseNumber = newObservingPoseNumber;
    }

    int KeyPointInfo::getInitObservingPoseNumber() const {
        return initialObservingPoseNumber;
    }
}
