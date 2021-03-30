//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/Point3d.h"

namespace gdr {


    Point3d::Point3d(const Eigen::Vector3d &pointEigenVectorXYZ, int newIndex) : x(pointEigenVectorXYZ[0]),
                                                                                 y(pointEigenVectorXYZ[1]),
                                                                                 z(pointEigenVectorXYZ[2]),
                                                                                 index(newIndex) {};

    Point3d::Point3d(double coordX, double coordY, double coordZ, int newIndex) : x(coordX),
                                                                                  y(coordY),
                                                                                  z(coordZ),
                                                                                  index(newIndex) {}

    Eigen::Vector4d Point3d::getEigenVector4dPointXYZ1() const {
        Eigen::Vector4d resultEigenVector4d;
        resultEigenVector4d.setOnes();
        resultEigenVector4d[0] = x;
        resultEigenVector4d[1] = y;
        resultEigenVector4d[2] = z;

        return resultEigenVector4d;
    }

    Eigen::Vector3d Point3d::getEigenVector3dPointXYZ() const {
        Eigen::Vector3d resultEigenVector3d;
        resultEigenVector3d.setOnes();
        resultEigenVector3d[0] = x;
        resultEigenVector3d[1] = y;
        resultEigenVector3d[2] = z;

        return resultEigenVector3d;
    }

    std::vector<double> Point3d::getVectorPointXYZ() const {
        std::vector<double> resultVectorXYZ = {x, y, z};
        return resultVectorXYZ;
    }

    int Point3d::getIndex() const {
        assert(index >= 0);
        return index;
    }

    std::vector<double> Point3d::getVectorPointXYZ1() const {
        std::vector<double> resultVectorXYZ = {x, y, z, 1};
        return resultVectorXYZ;
    }

    void Point3d::setNewCoordinates(double newX, double newY, double newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    void Point3d::setEigenVector3dPointXYZ(const Eigen::Vector3d &XYZ) {
        setNewCoordinates(XYZ[0], XYZ[1], XYZ[2]);
    }

    Point3d::Point3d(double coordX,
                     double coordY,
                     double coordZ) :
            x(coordX),
            y(coordY),
            z(coordZ) {}
}