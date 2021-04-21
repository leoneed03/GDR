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

    int SmoothPointCloud::registerPointCloudFromImages(const std::vector<Reconstructable> &posesToBeRegistered,
                                                       bool showVisualization,
                                                       float voxelSizeX,
                                                       float voxelSizeY,
                                                       float voxelSixeZ,
                                                       const std::string &pathPlyToSave,
                                                       const std::string &screenshotPath) {

        assert(!posesToBeRegistered.empty());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < posesToBeRegistered.size(); ++i) {

            const auto &rawPoints = posesToBeRegistered[i].getPointCloudXYZRGB();

            for (const auto &point: rawPoints) {
                pcl::PointXYZRGB pointToBeAdded;
                pointToBeAdded.x = point.getX();
                pointToBeAdded.y = point.getY();
                pointToBeAdded.z = point.getZ();

                pointToBeAdded.r = point.getR();
                pointToBeAdded.g = point.getG();
                pointToBeAdded.b = point.getB();

                input_cloud->push_back(pointToBeAdded);
            }


            if (i == posesToBeRegistered.size() - 1 || i % 10 == 0) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
                approximate_voxel_filter.setLeafSize(voxelSizeX, voxelSizeY, voxelSixeZ);
                approximate_voxel_filter.setInputCloud(input_cloud);
                approximate_voxel_filter.filter(*filtered_cloud);

                std::swap(filtered_cloud, input_cloud);
            }
        }

        if (!screenshotPath.empty()) {
            pcl::visualization::PCLVisualizer::Ptr viewer(
                    new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer->initCameraParameters();

            int v1(0);
            viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
            viewer->setBackgroundColor(0, 0, 0, v1);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(input_cloud);

            viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud, rgb, "for screenshot", v1);

            viewer->saveScreenshot(screenshotPath);
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