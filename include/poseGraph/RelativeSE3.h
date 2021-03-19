//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESSENTIALMATRIX_H
#define GDR_ESSENTIALMATRIX_H

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "poseGraph/VertexCG.h"

namespace gdr {

    class RelativeSE3 {

        const VertexCG &vertexFrom;
        const VertexCG &vertexTo;

        SE3 relativePose;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RelativeSE3(const SE3 &se3,
                    const VertexCG &vertexFrom,
                    const VertexCG &vertexTo);

        int getIndexTo() const;

        int getIndexFrom() const;

        Eigen::Quaterniond getRelativeRotation() const;

        Eigen::Vector3d getRelativeTranslation() const;

        Sophus::SE3d getRelativePoseSE3() const;

        const SE3 &getRelativePose() const;

    };
}
#endif
