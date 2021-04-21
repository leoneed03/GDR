//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "parametrization/Reconstructable.h"

#include "opencv2/opencv.hpp"

namespace gdr {

    Reconstructable::Reconstructable(const std::string &pathToRgbToSet,
                                     const std::string &pathToDepthToSet,
                                     const CameraRGBD &cameraRgbToSet) :
            pathToDepth(pathToDepthToSet),
            pathToRgb(pathToRgbToSet),
            cameraRgbd(cameraRgbToSet) {}

    std::vector<PointXYZRGBfloatUchar> Reconstructable::getPointCloudXYZRGB() const {
        double coeffDepth = getCamera().getDepthPixelDivider();

        cv::Mat depthImage = cv::imread(getPathDImage(), cv::IMREAD_ANYDEPTH);
        cv::Mat rgbImage = cv::imread(getPathRGBImage(), cv::IMREAD_ANYCOLOR);
        std::vector<PointXYZRGBfloatUchar> points;

        auto cameraToWorldSE3 = getAbsolutePose();

        for (int y = 0; y < depthImage.rows; ++y) {
            for (int x = 0; x < depthImage.cols; ++x) {

                //TODO: use depth to rgb SE3 to align images
                int currentKeypointDepth = depthImage.at<ushort>(y, x);

                double localZ = currentKeypointDepth / coeffDepth;

                std::vector<double> coordinatesPixelCenteredImage = {(x + 0.5),
                                                                     (y + 0.5),
                                                                     localZ,
                                                                     1.0};
                Eigen::Vector4d localCoordinatesXYZ1 = getCamera()
                        .getCoordinatesBeforeProjectionXYZ1(coordinatesPixelCenteredImage[0],
                                                            coordinatesPixelCenteredImage[1],
                                                            coordinatesPixelCenteredImage[2]);
                Eigen::Vector4d globalCoordinates = cameraToWorldSE3.getSE3().matrix()
                                                    * localCoordinatesXYZ1;

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

    double Reconstructable::getDepthMetersAt(const Point2d &point2D) const {
        assert(false && "not implemented yet");
        return 0;
    }

    const std::string &Reconstructable::getPathRGBImage() const {
        return pathToRgb;
    }

    const std::string &Reconstructable::getPathDImage() const {
        return pathToDepth;
    }

    const CameraRGBD &Reconstructable::getCamera() const {
        return cameraRgbd;
    }

    void Reconstructable::setAbsolutePose(const SE3 &cameraToWorldToSet) {

        cameraToWorld = cameraToWorldToSet;
    }

    const SE3 &Reconstructable::getAbsolutePose() const {

        return cameraToWorld;
    }
}