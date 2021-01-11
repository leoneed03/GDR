//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "pointCloud.h"
#include <opencv2/opencv.hpp>

namespace gdr {

    Eigen::Vector3d mirrorPoint(const Eigen::Vector3d &point, double mirrorParamH = 480, double mirrorParamW = 640) {
        Eigen::Vector3d newPoint = point;
        newPoint[0] = mirrorParamW - point[0];
        newPoint[1] = mirrorParamH - point[1];
        newPoint[2] = point[2];
        return newPoint;
    }
    Eigen::Vector4d mirrorPoint(const Eigen::Vector4d &point, double mirrorParamH = 480, double mirrorParamW = 640) {
        Eigen::Vector4d newPoint = point;
        newPoint[0] = mirrorParamW - point[0];
        newPoint[1] = mirrorParamH - point[1];
        newPoint[2] = point[2];
        newPoint[3] = 1;
        return newPoint;
    }

    cv::Mat getProjectedPointCloud(const std::string& pathToImageDepth, const Eigen::Matrix4d &transformation,
                                   const CameraRGBD &cameraRgbd) {
        return visualizeTransformedCloud(getPointCloudBeforeProjection(getPointCloudFromImage(pathToImageDepth), cameraRgbd), transformation, cameraRgbd);
    }
    cv::Mat visualizeTransformedCloud(const Eigen::Matrix4Xd &pointCloud, const Eigen::Matrix4d &transformation,
                                      const CameraRGBD &cameraRgbd) {

        int mirrorParamH = 480;
        int mirrorParamW = 640;
        cv::Mat emptyDepthImage = cv::Mat::zeros(cv::Size(mirrorParamW, mirrorParamH), CV_16UC1);
        Eigen::Matrix3Xd intrinsicsMatrix(3, 4);
        intrinsicsMatrix.setZero();
        intrinsicsMatrix.col(0)[0] = cameraRgbd.fx;
        intrinsicsMatrix.col(1)[1] = cameraRgbd.fy;
        intrinsicsMatrix.col(2)[0] = cameraRgbd.cx;
        intrinsicsMatrix.col(2)[1] = cameraRgbd.cy;
        intrinsicsMatrix.col(2)[2] = 1;
        std::cout << "intrinsics matrix \n" << intrinsicsMatrix << std::endl;
        Eigen::Matrix4Xd afterTransformation = transformation * pointCloud;
        Eigen::Matrix3Xd afterProjection = intrinsicsMatrix * afterTransformation;

        for (int i = 0; i < afterProjection.cols(); ++i) {

            const Eigen::Vector3d &point = afterProjection.col(i);
            Eigen::Vector3d newPoint = point;
            newPoint[0] /= point[2];
            newPoint[1] /= point[2];
            newPoint = mirrorPoint(newPoint);
            int x = newPoint[0];
            int y = newPoint[1];
            int depth = static_cast<int>(pointCloud.col(i)[2] * 5000.0);
            assert(depth <= std::numeric_limits<ushort>::max());
            if (x < 0 || x >= mirrorParamW || y < 0 || y >= mirrorParamH || depth <= 0) {
                continue;
            }

            auto oldDepth = emptyDepthImage.at<ushort>(y, x);
            if (oldDepth > 0) {
                emptyDepthImage.at<ushort>(y, x) = std::min(static_cast<ushort>(depth), oldDepth);
            } else {
                emptyDepthImage.at<ushort>(y, x) = static_cast<ushort>(depth);
            }
//            afterProjection.col(i) = newPoint;
        }
//        cv::imshow("After Transformation", emptyDepthImage);
//        cv::waitKey(0);
//        cv::destroyWindow("After Transformation");
        return emptyDepthImage;
    }


    std::vector<std::vector<double>> getPointCloudFromImage(const std::string& pathToImageDepth) {

        double coeffDepth = 5000.0;
        cv::Mat depthImage = cv::imread(pathToImageDepth, cv::IMREAD_ANYDEPTH);
        std::vector<std::vector<double>> points;

        for (int y = 0; y < depthImage.rows; ++y) {
            for (int x = 0; x < depthImage.cols; ++x) {
                int currentKeypointDepth = depthImage.at<ushort>(y, x);
                double globalX = x;
                double globalY = y;
                double globalZ = currentKeypointDepth / coeffDepth;

                if (currentKeypointDepth != 0) {
                    points.push_back({globalX, globalY, globalZ, 1});
                }
            }
        }
        return points;
    }

    /*
     * pointsFromImage -- Vector of "Points": {x, y, depth, 1} where x (column), y (row) are coordinates in OpenCV format
     * depth is actual depth in meters of a current point at position depthImage.at<ushort>(y, x)
     */
    Eigen::Matrix4Xd getPointCloudBeforeProjection(const std::vector<std::vector<double>> &pointsFromImage,
                                                   const CameraRGBD &cameraRgbd) {

        Eigen::Matrix4Xd points3D(4, pointsFromImage.size());

        for (int i = 0; i < pointsFromImage.size(); ++i) {
            Eigen::Vector4d point(pointsFromImage[i].data());

            ///mirror x, y coordinates if necessary
            Eigen::Vector4d newPoint = mirrorPoint(point);

            double oldX = newPoint[0];
            double oldY = newPoint[1];
            double oldZ = newPoint[2];

            double X = 1.0 * (oldX - cameraRgbd.cx) * oldZ / cameraRgbd.fx;
//            x1 = 1.0 * (x1 - cameraRgbd.cx) * z1 / cameraRgbd.fx;
            double Y = 1.0 * (oldY - cameraRgbd.cy) * oldZ / cameraRgbd.fy;
//            y1 = 1.0 * (y1 - cameraRgbd.cy) * z1 / cameraRgbd.fy;
            points3D.col(i) << X, Y, oldZ, 1;
        }

        return points3D;
    }
}