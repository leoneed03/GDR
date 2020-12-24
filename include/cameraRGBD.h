//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CAMERARGBD_H
#define GDR_CAMERARGBD_H

namespace gdr {

    struct CameraRGBD {

        float fx = 525.0;
        float fy = 525.0;
        float cx = 319.5;
        float cy = 239.5;

        CameraRGBD(float fx, float cx, float fy, float cy);
    };
}

#endif
