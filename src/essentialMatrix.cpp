#include "../include/essentialMatrix.h"

essentialMatrix::essentialMatrix(const cv::Mat &newInnerEssentialMatrix, const vertexCG &newVertexFrom,
                                 const vertexCG &vertexTo) : innerEssentialMatrix(newInnerEssentialMatrix),
                                                             vertexFrom(newVertexFrom), vertexTo(newVertexFrom) {}