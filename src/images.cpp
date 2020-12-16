#include <pcl/visualization/cloud_viewer.h>
#include "../include/images.h"

#include <pcl/point_types.h>

Cloud *parseDepthImageNoColour(const std::string &pathToDimage, const CameraRGBD &cameraRgbd) {

    cv::Mat depthImage = cv::imread(pathToDimage, cv::IMREAD_ANYDEPTH);
    pcl::PointCloud<pcl::PointXYZ> *cloud1 = new pcl::PointCloud<pcl::PointXYZ>();
    int num = 0;
    std::vector<std::vector<double>> points;

    for (uint x = 0; x < depthImage.cols; ++x) {
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
        cloud1->width = num_elements;
        cloud1->height = 1;
        cloud1->is_dense = false;
        cloud1->points.resize(cloud1->width * cloud1->height);

        for (size_t i = 0; i < num_elements; ++i) {
//            int r = 10;
//            int g = 10;
//            int b = 100;
//            int32_t rgb = (static_cast<uint32_t>(r) << 16 |
//                           static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
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



//////show
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud1);
//        viewer.showCloud(ptrCloud);
//
//
//        while (!viewer.wasStopped()) {
//        }

    }
}

//// pixels -- std::vector of <x, y, Intensity> where -height / 2 <= y <= height / 2, -width/2 <= x <= width/2, 0 <= Intensity <= 65536

/*

//// pixels -- std::vector of <x, y, Intensity> where 0 <= y <= height, 0 <= x <= width, 0 <= Intensity <= 65536


pangolin::Image<unsigned char> LoadImage(
        const std::vector<std::vector<int>> &pixels,
//        pangolin::PixelFormat& raw_fmt,
        size_t raw_width, size_t raw_height, size_t raw_pitch) {

    pangolin::PixelFormat raw_fmt = pangolin::PixelFormatFromString("GRAY16LE");
    unsigned char* a;
    pangolin::Image<unsigned char> img(raw_width, raw_height, raw_pitch, a);


    // Read from file, row at a time.
//    std::ifstream bFile(filename.c_str(), std::ios::in | std::ios::binary);

    int p = 0;
    for (size_t r = 0; r < img.h; ++r) {
//        bFile.read((char *) img.ptr + r * img.pitch, img.pitch);
        for (size_t c = 0; c < img.w; ++c) {
            int x, y, d;
            *(img.ptr + r * img.pitch + c) = 0;
            if (p < pixels.size()) {
                auto &pixel = pixels[p];
                x = pixel[0];
                y = pixel[1];
                d = pixel[2];
                if (x == c && y == r) {
                    *(img.ptr + r * img.pitch + c) = d;
                    ++p;
                }
            }
        }
    }
    return img;
}
pangolin::TypedImage LoadImageT(
        const std::vector<std::vector<int>> &pixels,
//        pangolin::PixelFormat& raw_fmt,
        size_t raw_width, size_t raw_height, size_t raw_pitch) {

    std::cout << "start loading" << std::endl;
    pangolin::PixelFormat raw_fmt = pangolin::PixelFormatFromString("GRAY16LE");
    pangolin::TypedImage img(raw_width, raw_height, raw_fmt, raw_pitch);


    // Read from file, row at a time.
//    std::ifstream bFile(filename.c_str(), std::ios::in | std::ios::binary);

    int p = 0;
    for (size_t r = 0; r < img.h; ++r) {
//        bFile.read((char *) img.ptr + r * img.pitch, img.pitch);

        for (size_t c = 0; c < img.w; ++c) {
            std::cout << "x = " << c << " y = " << r << std::endl;
            *(img.ptr + r * img.w + c) = 0;
        }
    }
    for (const auto& pixel: pixels) {
        int x, y, d;
        x = pixel[0];
        y = pixel[1];
        d = pixel[2];
        *(img.ptr + y * img.w + x + x) = d;

        *(img.ptr + y * img.w + x + x + 1) = d;
    }
    return img;
}






*/