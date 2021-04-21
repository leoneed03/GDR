//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RECONSTRUCTABLE_H
#define GDR_RECONSTRUCTABLE_H

#include <string>

#include "cameraModel/CameraRGBD.h"

#include "parametrization/SE3.h"
#include "parametrization/Point2d.h"
#include "parametrization/PointXYZRGBfloatUchar.h"

namespace gdr {

    class Reconstructable {

        SE3 cameraToWorld;
        std::string pathToRgb;
        std::string pathToDepth;
        CameraRGBD cameraRgbd;

    public:
        Reconstructable(const std::string &pathToRgbToSet,
                        const std::string &pathToDepthToSet,
                        const CameraRGBD &cameraRgbToSet);

        void setAbsolutePose(const SE3 &cameraToWorld);

        const SE3 &getAbsolutePose() const;

        std::vector<PointXYZRGBfloatUchar> getPointCloudXYZRGB() const;

        double getDepthMetersAt(const Point2d &point2D) const;

        const std::string &getPathRGBImage() const;

        const std::string &getPathDImage() const;

        const CameraRGBD &getCamera() const;
    };
}


#endif
