//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "poseGraph/RelativeSE3.h"


namespace gdr {

    int RelativeSE3::getIndexTo() const {
        return vertexTo.getIndex();
    }

    int RelativeSE3::getIndexFrom() const {
        return vertexFrom.getIndex();
    }

    Eigen::Quaterniond RelativeSE3::getRelativeRotation() const {
        return relativePose.getRotationQuatd();
    }

    Eigen::Vector3d RelativeSE3::getRelativeTranslation() const {
        return relativePose.getTranslation();
    }

    Sophus::SE3d RelativeSE3::getRelativePoseSE3() const {
        return relativePose.getSE3();
    }

    RelativeSE3::RelativeSE3(const SE3 &se3,
                             const VertexCG &newVertexFrom,
                             const VertexCG &newVertexTo):
                                                   vertexFrom(newVertexFrom),
                                                   vertexTo(newVertexTo),
                                                   relativePose(se3) {}

    const SE3& RelativeSE3::getRelativePose() const {
        return relativePose;
    }
}
