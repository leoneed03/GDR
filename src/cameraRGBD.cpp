//
// Copyright (c) Microsoft Corporation and contributors. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "../include/cameraRGBD.h"

CameraRGBD::CameraRGBD(float fx1, float cx1, float fy1, float cy1) : fx(fx1), fy(fy1), cx(cx1), cy(cy1) {
    cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}
