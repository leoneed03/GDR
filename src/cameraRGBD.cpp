//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "cameraRGBD.h"

namespace gdr {

    CameraRGBD::CameraRGBD(float fx1, float cx1, float fy1, float cy1) : fx(fx1), fy(fy1), cx(cx1), cy(cy1) {}

    Eigen::Matrix3Xd CameraRGBD::getIntrinsicsMatrix3x4() const {
        Eigen::Matrix3Xd intrinsicsMatrix(3, 4);
        intrinsicsMatrix.setZero();

        intrinsicsMatrix.col(0)[0] = fx;
        intrinsicsMatrix.col(1)[1] = fy;
        intrinsicsMatrix.col(2)[0] = cx;
        intrinsicsMatrix.col(2)[1] = cy;
        intrinsicsMatrix.col(2)[2] = 1;

        return intrinsicsMatrix;
    }

    Eigen::Matrix3d CameraRGBD::getIntrinsicsMatrix3x3() const {
        Eigen::Matrix3d intrinsicsMatrix;
        intrinsicsMatrix.setZero();

        intrinsicsMatrix.col(0)[0] = fx;
        intrinsicsMatrix.col(1)[1] = fy;
        intrinsicsMatrix.col(2)[0] = cx;
        intrinsicsMatrix.col(2)[1] = cy;
        intrinsicsMatrix.col(2)[2] = 1;

        return intrinsicsMatrix;
    }

}
