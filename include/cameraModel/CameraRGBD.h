//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CAMERARGBD_H
#define GDR_CAMERARGBD_H

#include <Eigen/Eigen>

#include "cameraModel/MeasurementErrorDeviationEstimators.h"
#include "parametrization/Point3d.h"

namespace gdr {

    class CameraRGBD {

        float fx = 525.0;
        float fy = 525.0;
        float cx = 319.5;
        float cy = 239.5;

        double depthPixelDivider = 5000.0;

        MeasurementErrorDeviationEstimators measurementErrorDeviationEstimators;


    public:

        void setMeasurementErrorDeviationEstimators(const MeasurementErrorDeviationEstimators &estimators);

        const MeasurementErrorDeviationEstimators &getMeasurementErrorDeviationEstimators() const;

        CameraRGBD() = default;

        double getDepthPixelDivider() const;

        void setDepthPixelDivider(double divider);

        CameraRGBD(float fx, float cx, float fy, float cy);

        Eigen::Matrix3Xd getIntrinsicsMatrix3x4() const;

        Eigen::Matrix3d getIntrinsicsMatrix3x3() const;

        Eigen::Vector3d getCoordinates3D(double xHorizontal, double yVertical, double depth) const;

        Eigen::Vector4d getCoordinatesBeforeProjectionXYZ1(double xHorizontal, double yVertical, double depth) const;

        Eigen::Vector4d getCoordinatesXYZ1BeforeProjectionXYZ1(const Eigen::Vector4d &pointXYZ1) const;

        Eigen::Matrix4Xd getPointCloudXYZ1BeforeProjection(const std::vector<Point3d> &pointsFromImageXYZ) const;

        float getFx() const;

        float getFy() const;

        float getCx() const;

        float getCy() const;
    };
}

#endif
