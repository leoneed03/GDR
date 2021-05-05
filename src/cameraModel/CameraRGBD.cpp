//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "cameraModel/CameraRGBD.h"

namespace gdr {

    CameraRGBD::CameraRGBD(double fx1, double cx1, double fy1, double cy1) : fx(fx1), fy(fy1), cx(cx1), cy(cy1) {}

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

    Eigen::Vector3d CameraRGBD::getCoordinates3D(double xHorizontal, double yVertical, double depth) const {

        double X = 1.0 * (xHorizontal - cx) * depth / fx;
        double Y = 1.0 * (yVertical - cy) * depth / fy;

        Eigen::Vector3d coordinates3D;
        coordinates3D[0] = X;
        coordinates3D[1] = Y;
        coordinates3D[2] = depth;

        return coordinates3D;
    }

    double CameraRGBD::getFx() const {
        return fx;
    }

    double CameraRGBD::getFy() const {
        return fy;
    }

    double CameraRGBD::getCx() const {
        return cx;
    }

    double CameraRGBD::getCy() const {
        return cy;
    }

    Eigen::Vector4d
    CameraRGBD::getCoordinatesBeforeProjectionXYZ1(double xHorizontal, double yVertical, double depth) const {
        Eigen::Vector4d coordinates4;
        coordinates4.setOnes();
        coordinates4.topLeftCorner<3, 1>() = getCoordinates3D(xHorizontal, yVertical, depth);

        return coordinates4;
    }

    Eigen::Matrix4Xd
    CameraRGBD::getPointCloudXYZ1BeforeProjection(const std::vector<Point3d> &pointsFromImageXYZ) const {
        Eigen::Matrix4Xd points3D(4, pointsFromImageXYZ.size());

        for (int i = 0; i < pointsFromImageXYZ.size(); ++i) {
            const auto &point = pointsFromImageXYZ[i];
            points3D.col(i) = getCoordinatesXYZ1BeforeProjectionXYZ1(point.getEigenVector4dPointXYZ1());
        }

        return points3D;
    }

    Eigen::Vector4d CameraRGBD::getCoordinatesXYZ1BeforeProjectionXYZ1(const Eigen::Vector4d &pointXYZ1) const {
        return getCoordinatesBeforeProjectionXYZ1(pointXYZ1[0], pointXYZ1[1], pointXYZ1[2]);
    }

    double CameraRGBD::getDepthPixelDivider() const {
//        assert(depthPixelDivider == 1000.0);
        return depthPixelDivider;
    }

    void CameraRGBD::setDepthPixelDivider(double divider) {
        depthPixelDivider = divider;
    }

    void CameraRGBD::setMeasurementErrorDeviationEstimators(const MeasurementErrorDeviationEstimators &estimators) {
        measurementErrorDeviationEstimators = estimators;
    }

    const MeasurementErrorDeviationEstimators &CameraRGBD::getMeasurementErrorDeviationEstimators() const {
        return measurementErrorDeviationEstimators;
    }

    void CameraRGBD::setDepthToRgbSe3(const SE3 &depthToRgbToSet) {
        depthToRgb = depthToRgbToSet;
    }

    const SE3 &CameraRGBD::getDepthToRgbSe3() const {
        return depthToRgb;
    }

}
