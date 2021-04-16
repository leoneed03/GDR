//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SE3_H
#define GDR_SE3_H

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include "parametrization/SO3.h"

namespace gdr {

    class SE3 {

    private:
        Sophus::SE3d se3;

    public:
        SE3() = default;

        SE3(const Sophus::SE3d &relativePoseSE3);

        explicit SE3(const Eigen::Matrix4d &poseMatrix4);

        void setRotation(const SO3 &orientationToSet);

        void setRotation(const Sophus::SO3d &orientationToSet);

        void setTranslation(const Eigen::Vector3d &translationToSet);

        Sophus::SE3d getSE3Constructed() const;

        const Sophus::SE3d &getSE3() const;

        Eigen::Vector3d getTranslation() const;

        Eigen::Quaterniond getRotationQuatd() const;

        Sophus::SO3d getSO3() const;

        SE3 inverse() const;

        std::pair<double, double> getRotationTranslationErrors(const SE3 &otherSe3);

        friend SE3 operator*(const SE3 &lhs, const SE3 &rhs);
    };
}

#endif
