#pragma once
#ifndef TEST_SIFTGPU_IMAGES_H
#define TEST_SIFTGPU_IMAGES_H

#include "cameraRGBD.h"
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

void parseDepthImage(const std::string& pathToDepthImage, const CameraRGBD& camera);

Cloud* parseDepthImageNoColour(const std::string& pathToDepthImage, const CameraRGBD& camera);

#endif
