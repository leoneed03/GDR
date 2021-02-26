//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVEPOSESE3_H
#define GDR_RELATIVEPOSESE3_H

#include "sophus/se3.hpp"

namespace gdr {
    struct RelativePoseSE3 {
        int indexFrom;
        int indexTo;
        Sophus::SE3d relativePoseSE3;

        RelativePoseSE3(int newIndexFrom, int newIndexTo, const Sophus::SE3d &newRelativePoseSE3);

        Sophus::SE3d getRelativePoseSE3Constructed() const;

        int getIndexFrom() const;

        int getIndexTo() const;

        const Sophus::SE3d& getRelativePoseSE3() const;

        Eigen::Vector3d getRelativeTranslationV3() const;

        Eigen::Quaterniond getRelativeRotationSO3Quatd() const;

        Eigen::Vector3d getRelativeTranslationR3() const;
    };
}

#endif
