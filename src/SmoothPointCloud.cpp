//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "SmoothPointCloud.h"
#include "pointCloud.h"


#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>

namespace gdr {

    struct PointXYZRGBdouble {
        double x, y, z;
        int R, G, B;

        PointXYZRGBdouble(double newX, double newY, double newZ, int newR, int newG, int newB) :
                x(newX), y(newY), z(newZ),
                R(newR), G(newG), B(newB) {};
    };

    std::vector<PointXYZRGBdouble> getPointCloudXYZRGBFromPose(const VertexCG &poseToBeRegistered) {
        double coeffDepth = 5000.0;
        cv::Mat depthImage = cv::imread(poseToBeRegistered.getPathDImage(), cv::IMREAD_ANYDEPTH);
        cv::Mat rgbImage = cv::imread(poseToBeRegistered.getPathRGBImage(), cv::IMREAD_ANYCOLOR);
        std::vector<PointXYZRGBdouble> points;

        Eigen::Matrix4d poseMatrix = poseToBeRegistered.getEigenMatrixAbsolutePose4d().inverse();


        for (int y = 0; y < depthImage.rows; ++y) {
            for (int x = 0; x < depthImage.cols; ++x) {
                int currentKeypointDepth = depthImage.at<ushort>(y, x);

                double width = 640;
                double height = 480;

                double localZ = currentKeypointDepth / coeffDepth;

                std::vector<double> coordinatesPixelCenteredImage = {(x + 0.5),
                                                                     (y + 0.5),
                                                                     localZ,
                                                                     1.0};
                auto localCoordinatesXYZ1 = getPointBeforeProjection(coordinatesPixelCenteredImage,
                                                                     poseToBeRegistered.getCamera());

                // ground truth
                Eigen::Vector4d globalCoordinates = poseMatrix.inverse() * localCoordinatesXYZ1;

                double globalX = -globalCoordinates[0];
                double globalY = -globalCoordinates[1];
                double globalZ = globalCoordinates[2];


                const auto &rgbInfo = rgbImage.at<cv::Vec3b>(y, x);


                if (currentKeypointDepth != 0) {

                    points.push_back({globalX, globalY, globalZ, rgbInfo[2], rgbInfo[1], rgbInfo[0]});
                }
            }
        }
        return points;
    }

    void SmoothPointCloud::registerPointCloudFromImage(const std::vector<VertexCG *> &posesToBeRegistered,
                                                       double voxelSizeX,
                                                       double voxelSizeY,
                                                       double voxelSixeZ) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < posesToBeRegistered.size(); ++i) {

            const auto &rawPoints = getPointCloudXYZRGBFromPose(*posesToBeRegistered[i]);

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

            std::cout << "Loaded _" << i << "_ " << input_cloud->size() << " data points from pose no "
                      << posesToBeRegistered[i]->getIndex()
                      << std::endl;

            if (i == posesToBeRegistered.size() - 1) {
                // Filtering input scan to roughly 10% of original size to increase speed of registration.
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
                approximate_voxel_filter.setLeafSize(voxelSizeX, voxelSizeY, voxelSixeZ);
                approximate_voxel_filter.setInputCloud(input_cloud);
                approximate_voxel_filter.filter(*filtered_cloud);
                std::cout << "                  Filtered cloud contains " << filtered_cloud->size()
                          << " data points from pose " << posesToBeRegistered[i]->getIndex() << std::endl;
                std::swap(filtered_cloud, input_cloud);
            }
        }
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(input_cloud);

        while (!viewer.wasStopped()) {
        }

//        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//        viewer->setBackgroundColor (0, 0, 0);
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(input_cloud);
//        viewer->addPointCloud<pcl::PointXYZRGB> (input_cloud, rgb, "sample cloud");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//        viewer->addCoordinateSystem (1.0);
//        viewer->initCameraParameters ();
//        while (!viewer->wasStopped ())
//        {
//        }

    }
}