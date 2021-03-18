//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSESE3_H
#define GDR_RELATIVEPOSESE3_H

//#include "sophus/se3.hpp"

#include "parametrization/SE3.h"

namespace gdr {
    struct RelativePoseSE3 {
        int indexFrom;
        int indexTo;
        SE3 relativePoseSE3;

        RelativePoseSE3(int indexFrom, int indexTo, const SE3 &relativePose);

        SE3 getRelativePoseSE3Constructed() const;

        int getIndexFrom() const;

        int getIndexTo() const;

        const SE3 &getRelativePoseSE3() const;

        Eigen::Vector3d getTranslation() const;

        Eigen::Quaterniond getRelativeRotationSO3Quatd() const;

        Sophus::SO3d getRotation() const;

    };
}

#endif
