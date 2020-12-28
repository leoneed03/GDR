//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "transformationRt.h"


namespace gdr {

    transformationRtMatrix::transformationRtMatrix(const Eigen::Matrix4d &newInnerEssentialMatrix,
                                                   const VertexCG &newVertexFrom,
                                                   const VertexCG &newVertexTo)
            : innerTranformationRtMatrix(newInnerEssentialMatrix),
              vertexFrom(newVertexFrom), vertexTo(newVertexTo) {

        auto dim = newInnerEssentialMatrix.cols() - 1;
        R = newInnerEssentialMatrix.block(0, 0, dim, dim);
        t = newInnerEssentialMatrix.block(0, dim, dim, 1);
    }
}
