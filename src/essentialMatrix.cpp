#include "../include/essentialMatrix.h"

essentialMatrix::essentialMatrix(const cv::Mat &newInnerEssentialMatrix, const vertexCG &newVertexFrom,
                                 const vertexCG &newVertexTo) : innerEssentialMatrix(newInnerEssentialMatrix),
                                                             vertexFrom(newVertexFrom), vertexTo(newVertexTo) {}