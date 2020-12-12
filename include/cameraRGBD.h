#ifndef SIFTGPU_CAMERARGBD_H
#define SIFTGPU_CAMERARGBD_H

#include "opencv2/opencv.hpp"
struct CameraRGBD {
    float fx = 525.0;
    float fy = 525.0;
    float cx = 319.5;
    float cy = 239.5;
    cv::Mat cameraMatrix;
    CameraRGBD(float fx, float cx, float fy, float cy);
};

#endif
