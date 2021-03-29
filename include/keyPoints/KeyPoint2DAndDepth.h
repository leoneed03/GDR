//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_KEYPOINT2DANDDEPTH_H
#define GDR_KEYPOINT2DANDDEPTH_H

#include <limits>

namespace gdr {
    class KeyPoint2DAndDepth {
        static constexpr double defValue = (std::numeric_limits<double>::lowest() / 2.0);
        double x, y;
        double scale = defValue;
        double orientation = defValue;
        double depth = defValue;

    public:

        KeyPoint2DAndDepth();

        void setDepth(double depth);

        double getDepth() const;

        bool isDepthUsable() const;

        KeyPoint2DAndDepth(double newX,
                           double newY,
                           double newScale,
                           double newOrientation);

        KeyPoint2DAndDepth(double newX, double newY);

        double getX() const;

        double getY() const;

        double getScale() const;

        double getOrientation() const;

        bool scaleOrientationAreUsable() const;

        bool isUsable() const;

        static double getDefValue();

    };
}


#endif
