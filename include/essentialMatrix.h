#pragma once

#ifndef TEST_SIFTGPU_ESSENTIALMATRIX_H
#define TEST_SIFTGPU_ESSENTIALMATRIX_H


#include <opencv2/opencv.hpp>
#include "vertexCG.h"

typedef struct essentialMatrix {
    cv::Mat innerEssentialMatrix;
    const vertexCG& vertexFrom;
    const vertexCG& vertexTo;
    essentialMatrix(const cv::Mat& newInnerEssentialMatrix, const vertexCG& newVertexFrom, const vertexCG& newVertexTo);
} essentialMatrix;

#endif
