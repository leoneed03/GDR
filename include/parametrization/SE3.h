//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_SE3_H
#define GDR_SE3_H

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

namespace gdr {
    class SE3 {
    private:
        Sophus::SE3d se3;
    public:
        SE3() = default;

        SE3(const Sophus::SE3d &newRelativePoseSE3);

        SE3(const Eigen::Matrix4d &poseMatrix4);

        Sophus::SE3d getSE3Constructed() const;

        const Sophus::SE3d &getSE3() const;

        Eigen::Vector3d getRelativeTranslationV3() const;

        Eigen::Quaterniond getRelativeRotationSO3Quatd() const;

        Sophus::SO3d getSO3() const;
    };
}

#endif
