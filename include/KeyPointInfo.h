//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINTINFO_H
#define GDR_KEYPOINTINFO_H

#include "SiftGPU.h"

#include <climits>

namespace gdr {
    struct KeyPointInfo {
//        SiftGPU::SiftKeypoint keypoint;
        double x, y, scale, orientation;
        double depth;
        int observingPoseNumber;

        double epsilonD = 1e-10;
//        double epsilonD = 3 * std::numeric_limits<double>::epsilon();

        KeyPointInfo();

        KeyPointInfo(const SiftGPU::SiftKeypoint& newKeypoint, double newDepth, int newObservingPoseNumber);


        KeyPointInfo(const KeyPointInfo& newKeypoint);

        double getX() const;
        double getY() const;
        double getScale() const;
        double getOrientation() const;
        double getDepth() const;

        int getObservingPoseNumber() const;


        KeyPointInfo& operator=(const KeyPointInfo& right);
        bool operator==(const KeyPointInfo& right);


    };
}

#endif
