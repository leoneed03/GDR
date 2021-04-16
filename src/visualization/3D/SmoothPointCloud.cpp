//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "visualization/3D/SmoothPointCloud.h"

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>

namespace gdr {


    std::vector<PointXYZRGBfloatUchar>
    SmoothPointCloud::getPointCloudXYZRGBFromPose(const VertexPose &poseToBeRegistered) {
        double coeffDepth = poseToBeRegistered.getCamera().getDepthPixelDivider();

        cv::Mat depthImage = cv::imread(poseToBeRegistered.getPathDImage(), cv::IMREAD_ANYDEPTH);
        cv::Mat rgbImage = cv::imread(poseToBeRegistered.getPathRGBImage(), cv::IMREAD_ANYCOLOR);
        std::vector<PointXYZRGBfloatUchar> points;

        Eigen::Matrix4d poseMatrix = poseToBeRegistered.getEigenMatrixAbsolutePose4d().inverse();


        for (int y = 0; y < depthImage.rows; ++y) {
            for (int x = 0; x < depthImage.cols; ++x) {
                int currentKeypointDepth = depthImage.at<ushort>(y, x);

                double localZ = currentKeypointDepth / coeffDepth;

                std::vector<double> coordinatesPixelCenteredImage = {(x + 0.5),
                                                                     (y + 0.5),
                                                                     localZ,
                                                                     1.0};
                Eigen::Vector4d localCoordinatesXYZ1 = poseToBeRegistered.getCamera()
                        .getCoordinatesBeforeProjectionXYZ1(coordinatesPixelCenteredImage[0],
                                                            coordinatesPixelCenteredImage[1],
                                                            coordinatesPixelCenteredImage[2]);
                Eigen::Vector4d globalCoordinates = poseMatrix.inverse() * localCoordinatesXYZ1;

                double globalX = -globalCoordinates[0];
                double globalY = -globalCoordinates[1];
                double globalZ = globalCoordinates[2];

                const auto &rgbInfo = rgbImage.at<cv::Vec3b>(y, x);

                if (currentKeypointDepth != 0) {
                    points.emplace_back(
                            PointXYZRGBfloatUchar(globalX, globalY, globalZ,
                                                  rgbInfo[2], rgbInfo[1], rgbInfo[0]));
                }
            }
        }
        return points;
    }


    int SmoothPointCloud::registerPointCloudFromImages(const std::vector<VertexPose> &posesToBeRegistered,
                                                       bool showVisualization,
                                                       float voxelSizeX,
                                                       float voxelSizeY,
                                                       float voxelSixeZ,
                                                       const std::string &pathPlyToSave) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < posesToBeRegistered.size(); ++i) {

            const auto &rawPoints = getPointCloudXYZRGBFromPose(posesToBeRegistered[i]);

            for (const auto &point: rawPoints) {
                pcl::PointXYZRGB pointToBeAdded;
                pointToBeAdded.x = point.x;
                pointToBeAdded.y = point.y;
                pointToBeAdded.z = point.z;

                pointToBeAdded.r = point.R;
                pointToBeAdded.g = point.G;
                pointToBeAdded.b = point.B;

                input_cloud->push_back(pointToBeAdded);
            }


            if (i == posesToBeRegistered.size() - 1 || i % 5 == 0) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
                approximate_voxel_filter.setLeafSize(voxelSizeX, voxelSizeY, voxelSixeZ);
                approximate_voxel_filter.setInputCloud(input_cloud);
                approximate_voxel_filter.filter(*filtered_cloud);

                std::swap(filtered_cloud, input_cloud);
            }
        }


        if (showVisualization) {
            pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
            viewer.showCloud(input_cloud);

            while (!viewer.wasStopped()) {
            }
        }


        if (!pathPlyToSave.empty()) {
            pcl::PLYWriter plyWriter;
            pcl::PCLPointCloud2 readablePointCloud;
            pcl::toPCLPointCloud2(*input_cloud, readablePointCloud);

            ((pcl::FileWriter *) &plyWriter)->write(pathPlyToSave, readablePointCloud);
        }

        return 0;
    }
}