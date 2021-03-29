//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_ICLOUDPROJECTOR_H
#define GDR_ICLOUDPROJECTOR_H

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include "keyPoints/KeyPointInfo.h"
#include "parametrization/Point3d.h"

namespace gdr {

    class ICloudProjector {
        
    public:
        /**
         * @param indexedPoint is observed point number
         * @param poseNumberAndProjectedKeyPointInfo contains information about
         *      which poses observe specific point and Sift keypoint info like x, y, scale and orientation
         */
        virtual int addPoint(int indexedPoint,
                             const std::vector<KeyPointInfo> &poseNumberAndProjectedKeyPointInfo) = 0;

        /**
         * Computes global coordinates for all the observed points
         * @returns vector of computed points
         */
        virtual std::vector<Point3d> computedPointsGlobalCoordinates() = 0;

        /**
         * Gets information about all the observed points and cameras
         * @returns vector of maps, where each map returns keypoint information by keypoint's class
         */
        virtual const std::vector<std::unordered_map<int, KeyPointInfo>> &
        getKeyPointInfoByPoseNumberAndPointClass() const = 0;


        // TODO: further methods should be in a separate class
        /**
         * Shows where observed point are projected on images (for debug purposes)
         */
        virtual std::vector<cv::Mat> showPointsReprojectionError(const std::vector<Point3d> &pointsGlobalCoordinates,
                                                                 const std::string &pathToRGBDirectoryToSave,
                                                                 std::vector<double> &totalL2Errors,
                                                                 const CameraRGBD &camerasFromTo,
                                                                 int maxPointsToShow = -1,
                                                                 bool drawCirclesKeyPoints = false,
                                                                 double quantil = 0.5) const = 0;

        /**
         * Set new computed camera coordinates
         */
        virtual void setPoses(const std::vector<SE3> &poses) = 0;

        /**
        * Set new computed points coordinates
        */
        virtual void setPoints(const std::vector<Point3d> &points) = 0;

        virtual ~ICloudProjector() = default;
    };

}

#endif
