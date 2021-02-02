//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "ImageDrawer.h"

#include <opencv2/opencv.hpp>

namespace gdr {

    int ImageDrawer::showKeyPointOnImage(const std::string &pathToRGBImage, const KeyPointInfo &keyPointInfo,
                                         int pointIndex, std::string pathToSave, std::string nameToSave) {

        cv::Mat imageNoKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);
        cv::KeyPoint keyPointToShow(keyPointInfo.getX(), keyPointInfo.getY(), keyPointInfo.scale);

        cv::Mat imageWithKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);
        cv::drawKeypoints(imageNoKeyPoint, {keyPointToShow}, imageWithKeyPoint);


//        cv::namedWindow( );
        cv::imshow("Showing " + pathToRGBImage + " " + std::to_string(pointIndex), imageWithKeyPoint);
        cv::waitKey(0);
        return 0;
    }

    // each pair represents keyPoint's index (global class number) and info about it being a projection
    //

    int ImageDrawer::showKeyPointsOnImage(const std::string &pathToRGBImage,
                                          const std::vector<std::pair<int, KeyPointInfo>> &keyPointInfos,
                                          int maxIndexKeyPointToShow,
                                          std::string pathToSave,
                                          std::string nameToSave) {

        cv::Mat imageNoKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);
        std::vector<cv::KeyPoint> keyPointsToShow;


        cv::Mat imageWithKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);

        for (const auto &pairedIndexAndInfo: keyPointInfos) {
            const auto &keyPointInfo = pairedIndexAndInfo.second;
            double x = keyPointInfo.getX();
            double y = keyPointInfo.getY();
            cv::KeyPoint keyPointToShow(x, y, keyPointInfo.scale);


            int currentIndexKeyPoint = pairedIndexAndInfo.first;

            if (currentIndexKeyPoint > maxIndexKeyPointToShow) {
//                continue;
            }
            keyPointsToShow.push_back(keyPointToShow);
            cv::putText(imageWithKeyPoint, //target image
                        std::to_string(currentIndexKeyPoint), //text
                        cv::Point(x + 2 * std::numeric_limits<double>::epsilon(), y), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        0.2,
                        CV_RGB(118, 185, 0), //font color
                        1);
        }

        cv::drawKeypoints(imageNoKeyPoint, {keyPointsToShow}, imageWithKeyPoint);

        for (const auto &pairedIndexAndInfo: keyPointInfos) {

            const auto &keyPointInfo = pairedIndexAndInfo.second;
            double x = keyPointInfo.getX();
            double y = keyPointInfo.getY();
            int currentIndexKeyPoint = pairedIndexAndInfo.first;

            if (currentIndexKeyPoint > maxIndexKeyPointToShow) {
//                continue;
            }
            cv::putText(imageWithKeyPoint, //target image
                        std::to_string(currentIndexKeyPoint), //text
                        cv::Point(x, y), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        0.4,
                        CV_RGB(118, 185, 0), //font color
                        1);

        }

//        cv::imshow( "Showing " + pathToRGBImage + " ", imageWithKeyPoint);
//        cv::waitKey(0);

        if (pathToSave.empty()) {
            return 0;
        }
        std::string pathToSaveWithName = pathToSave + "/" + nameToSave;
        cv::imwrite(pathToSaveWithName, imageWithKeyPoint);
        return 0;
    }
}
