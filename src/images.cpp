#include <pcl/visualization/cloud_viewer.h>
#include "../include/images.h"

#include <pcl/point_types.h>

Cloud *parseDepthImageNoColour(const std::string &pathToDimage, const CameraRGBD &cameraRgbd) {
    cv::Mat depthImage = cv::imread(pathToDimage, cv::IMREAD_ANYDEPTH);

    pcl::PointCloud<pcl::PointXYZ> *cloud1 = new pcl::PointCloud<pcl::PointXYZ>();
    int num = 0;
    std::vector<std::vector<double>> points;
    for (uint x = 0; x < depthImage.cols; ++x) {
//            std::cout << std::setw(7) << x << ":";
//            myfile << std::setw(7) << x << ":";
        for (uint y = 0; y < depthImage.rows; ++y) {
            auto currentDepth = depthImage.ptr<ushort>(y)[x];
            assert(currentDepth == depthImage.at<ushort>(y, x));
            if (currentDepth == 0) {
                continue;
            }

            double x1, y1, z1;

            x1 = x + 0.5;
            y1 = y + 0.5;
            z1 = currentDepth / 5000.0;

            z1 = z1;
            x1 = 1.0 * (x1 - cameraRgbd.cx) * z1 / cameraRgbd.fx;
            y1 = 1.0 * (y1 - cameraRgbd.cy) * z1 / cameraRgbd.fy;
            points.push_back({x1, y1, z1});

        }
//            std::cout << std::endl;
    }
    if (true) {


        int num_elements = points.size();
        cloud1->width = num_elements;
        cloud1->height = 1;
        cloud1->is_dense = false;
        cloud1->points.resize(cloud1->width * cloud1->height);

        for (size_t i = 0; i < num_elements; ++i) {
            int r = 10;
            int g = 10;
            int b = 100;
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            cloud1->points[i].x = points[i][0];
            cloud1->points[i].y = points[i][1];
            cloud1->points[i].z = points[i][2];
//            cloud1->points[i].rgb = rgb;
        }
    }
//
//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud1);
//        viewer.showCloud(ptrCloud);
//
//
//        while (!viewer.wasStopped()) {
//        }
////
    assert(cloud1->width > 0);
    std::cout << "size inside " << cloud1->size() << std::endl;
    return cloud1;
}
typedef pcl::PointXYZRGBA PointT;
void parseDepthImage(const std::string &pathToDimage, const CameraRGBD &cameraRgbd) {

    cv::Mat depthImage = cv::imread(pathToDimage, cv::IMREAD_ANYDEPTH);

    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
    int num = 0;
    std::vector<std::vector<double>> points;
    for (uint x = 0; x < depthImage.cols; ++x) {
//            std::cout << std::setw(7) << x << ":";
//            myfile << std::setw(7) << x << ":";
        for (uint y = 0; y < depthImage.rows; ++y) {
            auto currentDepth = depthImage.ptr<ushort>(y)[x];
            assert(currentDepth == depthImage.at<ushort>(y, x));
            if (currentDepth == 0) {
                continue;
            }

            double x1, y1, z1;


            x1 = x + 0.5;
            y1 = y + 0.5;
            assert(y1 > 0 && y1 < depthImage.rows);
            assert(depthImage.rows == 480);
            y1 = depthImage.rows - y1;

            assert(x1 > 0 && x1 < depthImage.cols);
            assert(depthImage.cols == 640);
            x1 = depthImage.cols - x1;

            z1 = currentDepth / 5000.0;

            z1 = z1;
            x1 = 1.0 * (x1 - cameraRgbd.cx) * z1 / cameraRgbd.fx;
            y1 = 1.0 * (y1 - cameraRgbd.cy) * z1 / cameraRgbd.fy;
            points.push_back({x1, y1, z1});

        }
//            std::cout << std::endl;
    }
    if (true) {


        int num_elements = points.size();
        cloud1.width = num_elements;
        cloud1.height = 1;
        cloud1.is_dense = false;
        cloud1.points.resize(cloud1.width * cloud1.height);

        for (size_t i = 0; i < num_elements; ++i) {
            int r = 10;
            int g = 10;
            int b = 100;
            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            cloud1.points[i].x = points[i][0];
            cloud1.points[i].y = points[i][1];
            cloud1.points[i].z = points[i][2];
            cloud1.points[i].rgb = rgb;
        }

        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        PointT point;
//
//        point.getArray3fMap() << 1, 0, 0;
//        viewer.addText3D("x", point, 0.2, 1, 0, 0, "x_");
//
//        point.getArray3fMap() << 0, 1, 0;
//        viewer.addText3D("y", point, 0.2, 0, 1, 0, "y_");
//
//        point.getArray3fMap() << 0, 0, 1;
//        viewer.addText3D("z ", point, 0.2, 0, 0, 1, "z_");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud1);
        viewer.showCloud(ptrCloud);


        while (!viewer.wasStopped()) {
        }

    }
}



