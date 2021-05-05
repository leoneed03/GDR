//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CAMERARGBD_H
#define GDR_CAMERARGBD_H

#include <Eigen/Eigen>

#include "cameraModel/MeasurementErrorDeviationEstimators.h"
#include "parametrization/Point3d.h"
#include "parametrization/SE3.h"

namespace gdr {

    class CameraRGBD {

        double fx = 525.0;
        double fy = 525.0;
        double cx = 319.5;
        double cy = 239.5;

        double depthPixelDivider = 5000.0;

        MeasurementErrorDeviationEstimators measurementErrorDeviationEstimators;

        SE3 depthToRgb;


    public:

        template<class T>
        Sophus::Vector<T, 2> projectUsingIntrinsics(const Sophus::Vector<T, 3> &point) const {

            Sophus::Vector<T, 2> projectedPoint;

            projectedPoint[0] = T(getFx()) * point[0] + T(getCx()) * point[2];
            projectedPoint[1] = T(getFy()) * point[1] + T(getCy()) * point[2];

            projectedPoint[0] /= point[2];
            projectedPoint[1] /= point[2];

            return projectedPoint;
        }

        void setMeasurementErrorDeviationEstimators(const MeasurementErrorDeviationEstimators &estimators);

        const MeasurementErrorDeviationEstimators &getMeasurementErrorDeviationEstimators() const;

        CameraRGBD() = default;

        double getDepthPixelDivider() const;

        void setDepthPixelDivider(double divider);

        CameraRGBD(double fx, double cx, double fy, double cy);

        Eigen::Matrix3Xd getIntrinsicsMatrix3x4() const;

        Eigen::Matrix3d getIntrinsicsMatrix3x3() const;

        Eigen::Vector3d getCoordinates3D(double xHorizontal, double yVertical, double depth) const;

        Eigen::Vector4d getCoordinatesBeforeProjectionXYZ1(double xHorizontal, double yVertical, double depth) const;

        Eigen::Vector4d getCoordinatesXYZ1BeforeProjectionXYZ1(const Eigen::Vector4d &pointXYZ1) const;

        Eigen::Matrix4Xd getPointCloudXYZ1BeforeProjection(const std::vector<Point3d> &pointsFromImageXYZ) const;

        double getFx() const;

        double getFy() const;

        double getCx() const;

        double getCy() const;

        void setDepthToRgbSe3(const SE3 &depthToRgbToSet);

        const SE3 &getDepthToRgbSe3() const;
    };
}

#endif
