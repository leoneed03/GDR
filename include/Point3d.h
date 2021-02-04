//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_POINT3D_H
#define GDR_POINT3D_H

#include <Eigen/Eigen>

namespace gdr {

    struct Point3d {

        int index = -1;
        double x;
        double y;
        double z;

        Point3d() = delete;

        Point3d(const Eigen::Vector3d& pointEigenVectorXYZ, int newIndex);

        Point3d(double coordX, double coordY, double coordZ, int newIndex);

        void setNewCoordinates(double newX, double newY, double newZ);


        void setEigenVector3dPointXYZ(const Eigen::Vector3d& XYZ);

        Eigen::Vector4d getEigenVector4dPointXYZ1() const;

        Eigen::Vector3d getEigenVector3dPointXYZ() const;

        std::vector<double> getVectorPointXYZ() const;

        std::vector<double> getVectorPointXYZ1() const;

        int getIndex() const;


    };
}

#endif
