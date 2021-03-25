//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#ifndef GDR_CLOUDPROJECTOR_H
#define GDR_CLOUDPROJECTOR_H

#include "parametrization/Point3d.h"
#include "keyPoints/KeyPointInfo.h"
#include "sparsePointCloud/PointClassifier.h"
#include "parametrization/cameraRGBD.h"
#include "ProjectableInfo.h"

#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>

#include "sparsePointCloud/ICloudProjector.h"

namespace gdr {

    class CloudProjector : public ICloudProjector {

    public:

        /**
         * @param cameraPoses contains information about observing poses
         */
        explicit CloudProjector(const std::vector<ProjectableInfo> &cameraPoses);

        /**
         * @param indexedPoint is observed point number
         * @param poseNumberAndProjectedKeyPointInfo contains information about
         *      which poses observe specific point and Sift keypoint info like x, y, scale and orientation
         */
        int addPoint(int indexedPoint,
                     const std::vector<KeyPointInfo> &poseNumberAndProjectedKeyPointInfo) override;

        /**
         * Computes global coordinates for all the observed points
         * @returns vector of computed points
         */
        std::vector<Point3d> setComputedPointsGlobalCoordinates() override;

        /**
         * Gets information about all the observed points and cameras
         * @returns vector of maps, where each map returns keypoint information by keypoint's class
         */
        const std::vector<std::unordered_map<int, KeyPointInfo>> &getKeyPointInfoByPoseNumberAndPointClass() const override;

        /**
         * Shows where observed point is on images (for debug purposes)
         */
        std::vector<cv::Mat> showPointsReprojectionError(const std::vector<Point3d> &pointsGlobalCoordinates,
                                                         const std::string &pathToRGBDirectoryToSave,
                                                         std::vector<double> &totalL2Errors,
                                                         const CameraRGBD &camerasFromTo,
                                                         int maxPointsToShow = -1,
                                                         bool drawCirclesKeyPoints = false,
                                                         double quantil = 0.5) const override;

        void setPoses(const std::vector<SE3>& poses) override;

        void setPoints(const std::vector<Point3d> &points) override;

    private:
        int maxPointIndex = -1;
        std::vector<Point3d> indexedPoints;
        std::vector<ProjectableInfo> poses;

        // i-th unordered map maps from point's index (int) to struct containing information
        // about keypoint (KeyPointInfo) -- point's observation by i-th camera
        std::vector<std::unordered_map<int, KeyPointInfo>> keyPointInfoByPose;

        // j-th vector contains pose numbers observing j-keypoint
        std::vector<std::vector<int>> numbersOfPosesObservingSpecificPoint;

        const Point3d &getPointByIndex3d(int pointNumber3d) const;

        int getPoseNumber() const;

        std::vector<std::pair<int, KeyPointInfo>> getKeyPointsIndicesAndInfoByPose(int poseNumber) const;

    };
}

#endif
