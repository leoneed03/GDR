//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTINFO_H
#define GDR_KEYPOINTINFO_H

#include <limits>

#include "KeyPoint2DAndDepth.h"

namespace gdr {

    class KeyPointInfo {

        KeyPoint2DAndDepth keyPoint2D;

        int observingPoseNumber = -1;
        int initialObservingPoseNumber = -1;

        double epsilonD = 10 * std::numeric_limits<double>::epsilon();

    public:
        KeyPointInfo() = default;

        KeyPointInfo(const KeyPoint2DAndDepth &keyPointToSet,
                     int observingPoseNumberToSet);

        int getInitObservingPoseNumber() const;

        void setObservingPoseNumber(int newObservingPoseNumber);

        double getDefValue() const;

        double getX() const;

        double getY() const;

        double getScale() const;

        double getOrientation() const;

        double getDepth() const;

        int getObservingPoseNumber() const;

        bool isInitialized() const;

        bool operator==(const KeyPointInfo &right);

    };
}

#endif
