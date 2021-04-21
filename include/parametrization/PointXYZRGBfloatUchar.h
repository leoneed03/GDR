//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINTXYZRGBFLOATUCHAR_H
#define GDR_POINTXYZRGBFLOATUCHAR_H

namespace gdr {

    class PointXYZRGBfloatUchar {
        float x, y, z;
        unsigned char R, G, B;

    public:
        PointXYZRGBfloatUchar(float x, float y, float z,
                              unsigned char R, unsigned char G, unsigned char B);

        float getX() const;

        float getY() const;

        float getZ() const;

        unsigned char getR() const;

        unsigned char getG() const;

        unsigned char getB() const;
    };
}

#endif
