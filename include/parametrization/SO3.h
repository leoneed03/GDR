//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SO3_H
#define GDR_SO3_H

#include <sophus/so3.hpp>
#include <iostream>

namespace gdr {

    class SO3 {

    private:
        Sophus::SO3d rotationInner;

        const static int spaceIomanip = 15;

        int getSpaceIO() const;

    public:
        SO3 inverse() const;

        SO3() = default;

        SO3(const Sophus::SO3d &rotationSophus);

        SO3(const Eigen::Matrix3d &rotationEigenMatrix);

        SO3(const Eigen::Quaterniond &rotationEigenQuat);

        Eigen::Vector3d getLog() const;

        Eigen::Quaterniond getUnitQuaternion() const;

        const Sophus::SO3d &getRotationSophus() const;

        static Eigen::Matrix3d getRandomRotationMatrix3d();

        static Eigen::Quaterniond getRandomUnitQuaternion();


        friend std::vector<SO3> operator*(const SO3 &so3, const std::vector<SO3> &rotations);

        friend SO3 operator*(const SO3 &lhs, const SO3 &rhs);

        friend std::ostream &operator<<(std::ostream &os, const SO3 &rotation3D);
    };
}
#endif
