//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CAMERARGBD_H
#define GDR_CAMERARGBD_H

#include <Eigen/Eigen>

namespace gdr {

    struct CameraRGBD {

        float fx = 525.0;
        float fy = 525.0;
        float cx = 319.5;
        float cy = 239.5;

        CameraRGBD() = default;

        CameraRGBD(float fx, float cx, float fy, float cy);

        Eigen::Matrix3Xd getIntrinsicsMatrix3x4() const;

        Eigen::Matrix3d getIntrinsicsMatrix3x3() const;

        Eigen::Vector3d getCoordinates3D(double xHorizontal, double yVertical, double depth) const;

        float getFx() const;
        float getFy() const;
        float getCx() const;
        float getCy() const;
    };
}

#endif
