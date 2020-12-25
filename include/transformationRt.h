//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ESSENTIALMATRIX_H
#define GDR_ESSENTIALMATRIX_H

#include <opencv2/opencv.hpp>

#include "VertexCG.h"

namespace gdr {

    struct transformationRtMatrix {

        Eigen::Matrix4d innerTranformationRtMatrix;
        const VertexCG &vertexFrom;
        const VertexCG &vertexTo;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        transformationRtMatrix(const Eigen::Matrix4d &newInnerEssentialMatrix, const VertexCG &newVertexFrom,
                               const VertexCG &newVertexTo);
    };
}
#endif
