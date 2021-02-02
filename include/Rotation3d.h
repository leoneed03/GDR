//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ROTATION3D_H
#define GDR_ROTATION3D_H


#include <sophus/se3.hpp>
#include <iostream>

namespace gdr {

    struct Rotation3d {

    private:
        Sophus::SO3d rotationInner;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        Rotation3d() = delete;

//        Rotation3d(const Sophus::SO3d &rotationSophus);

        Rotation3d(const Eigen::Matrix3d &rotationEigenMatrix);

        Rotation3d(const Eigen::Quaterniond &rotationEigenQuat);

        Eigen::Vector3d getLog() const;

        Eigen::Quaterniond getUnitQuaternion() const;

        const Sophus::SO3d &getRotationSophus() const;

        static Eigen::Matrix3d getRandomRotationMatrix3d();

        static Eigen::Quaterniond getRandomUnitQuaternion();

        Rotation3d& operator=(const Rotation3d& right);

        friend std::ostream& operator<<(std::ostream& os, const Rotation3d& rotation3D);


    };
}
#endif
