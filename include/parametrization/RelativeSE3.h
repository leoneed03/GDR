//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_RELATIVE_SE3_H
#define GDR_RELATIVE_SE3_H

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "poseGraph/VertexCG.h"

namespace gdr {

    class RelativeSE3 {

        int indexFromDestination;
        int indexToToBeTransformed;

        SE3 relativePose;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RelativeSE3(int indexFromDestinationToSet,
                    int indexToToBeTransformedToSet,
                    const SE3 &se3);

        int getIndexTo() const;

        int getIndexFrom() const;

        SO3 getRelativeRotation() const;

        Eigen::Vector3d getRelativeTranslation() const;

        Sophus::SE3d getRelativePoseSE3() const;

        const SE3 &getRelativePose() const;

    };
}
#endif
