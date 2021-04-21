//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINT2D_H
#define GDR_POINT2D_H

namespace gdr {

    class Point2d {
        double x;
        double y;

    public:
        Point2d(double x, double y);

        double getX() const;

        double getY() const;
    };

}


#endif
