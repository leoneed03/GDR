#pragma once

#ifndef TEST_SIFTGPU_ESSENTIALMATRIX_H
#define TEST_SIFTGPU_ESSENTIALMATRIX_H


#include <opencv2/opencv.hpp>
#include "vertexCG.h"

typedef struct essentialMatrix {
    cv::Mat innerEssentialMatrix;
    const vertexCG& vertexFrom;
    const vertexCG& vertexTo;
    cv::Mat R;
    cv::Mat t;
    essentialMatrix(const cv::Mat& newInnerEssentialMatrix, const vertexCG& newVertexFrom, const vertexCG& newVertexTo, const cv::Mat& newR, const cv::Mat& newT);
} essentialMatrix;

#endif
