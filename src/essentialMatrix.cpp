#include "../include/essentialMatrix.h"

essentialMatrix::essentialMatrix(const MatrixX &newInnerEssentialMatrix, const vertexCG &newVertexFrom,
                                 const vertexCG &newVertexTo, const MatrixX& newR, const MatrixX& newT) : innerEssentialMatrix(newInnerEssentialMatrix),
                                                             vertexFrom(newVertexFrom), vertexTo(newVertexTo), R(newR), t(newT) {
    auto dim = newInnerEssentialMatrix.cols() - 1;
    R = newInnerEssentialMatrix.block(0, 0, dim, dim);
    t = newInnerEssentialMatrix.block(0, dim, dim, 1);
}