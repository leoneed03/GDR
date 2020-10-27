#include "../include/essentialMatrix.h"

essentialMatrix::essentialMatrix(const cv::Mat &newInnerEssentialMatrix, const vertexCG &newVertexFrom,
                                 const vertexCG &newVertexTo, const cv::Mat& newR, const cv::Mat& newT) : innerEssentialMatrix(newInnerEssentialMatrix),
                                                             vertexFrom(newVertexFrom), vertexTo(newVertexTo), R(newR), t(newT) {}