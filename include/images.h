#pragma once
#ifndef TEST_SIFTGPU_IMAGES_H
#define TEST_SIFTGPU_IMAGES_H

#include "cameraRGBD.h"
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

//
//#include <pangolin/image/image_io.h>
//#include <pangolin/image/typed_image.h>
//#include <ICPOdometry.h>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;
//pangolin::Image<unsigned char> LoadImage(
//        const std::vector<std::vector<int>>& pixels,
////        const pangolin::PixelFormat& raw_fmt,
//        size_t raw_width, size_t raw_height, size_t raw_pitch);
//pangolin::TypedImage LoadImageT(
//        const std::vector<std::vector<int>>& pixels,
////        const pangolin::PixelFormat& raw_fmt,
//        size_t raw_width, size_t raw_height, size_t raw_pitch);

void parseDepthImage(const std::string& pathToDepthImage, const CameraRGBD& camera);

Cloud* parseDepthImageNoColour(const std::string& pathToDepthImage, const CameraRGBD& camera);

#endif
