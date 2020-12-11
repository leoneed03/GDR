#include "../include/transformationRt.h"

transformationRtMatrix::transformationRtMatrix(const MatrixX &newInnerEssentialMatrix, const VertexCG &newVertexFrom,
                                               const VertexCG &newVertexTo, const MatrixX& newR, const MatrixX& newT) : innerTranformationRtMatrix(newInnerEssentialMatrix),
                                                                                                                        vertexFrom(newVertexFrom), vertexTo(newVertexTo), R(newR), t(newT) {
    auto dim = newInnerEssentialMatrix.cols() - 1;
    R = newInnerEssentialMatrix.block(0, 0, dim, dim);
    t = newInnerEssentialMatrix.block(0, dim, dim, 1);
}