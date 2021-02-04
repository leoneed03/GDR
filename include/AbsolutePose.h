//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ABSOLUTEPOSE_H
#define GDR_ABSOLUTEPOSE_H

#include "Rotation3d.h"

#include <sophus/se3.hpp>

namespace gdr {
    struct AbsolutePose {
        Sophus::SE3d absolutePoseSE3;

        AbsolutePose() = default;

        AbsolutePose(const Sophus::SE3d& poseSE3);

        AbsolutePose(const Eigen::Matrix4d& poseMatrix4d);

        AbsolutePose(const Eigen::Quaterniond& rotationQuatd, const Eigen::Vector3d& translationVector3d);

        Sophus::SE3d getSophusSE3d() const;

        Sophus::SO3d getRotationSO3() const;

        Eigen::Vector3d getTranslationVector3d() const;

        Rotation3d getRotation3d() const;
    };
}

#endif
