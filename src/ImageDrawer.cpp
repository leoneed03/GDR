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

    int ImageDrawer::showKeyPointMatchesTwoImages(const std::string &pathToRGBImageFirst,
                                                  const std::vector<KeyPointInfo> &keyPointInfosFirstImage,
                                                  const std::string &pathToRGBImageSecond,
                                                  const std::vector<KeyPointInfo> &keyPointInfosSecondImage,
                                                  const std::vector<std::pair<int, int>> &matchesKeypoints,
                                                  int maxIndexKeyPointToShow,
                                                  std::string pathToSave) {

        // first image with keypoints
        cv::Mat imageNoKeyPointFirst = cv::imread(pathToRGBImageFirst, cv::IMREAD_COLOR);
        std::vector<cv::KeyPoint> keyPointsToShowFirst;
        for (const auto& keyPointInfo: keyPointInfosFirstImage) {
            cv::KeyPoint keyPointToShow(keyPointInfo.getX(), keyPointInfo.getY(), keyPointInfo.getScale());
            keyPointsToShowFirst.emplace_back(keyPointToShow);
        }
        // second image and keypoints
        cv::Mat imageNoKeyPointSecond = cv::imread(pathToRGBImageSecond, cv::IMREAD_COLOR);
        std::vector<cv::KeyPoint> keyPointsToShowSecond;
        for (const auto& keyPointInfo: keyPointInfosSecondImage) {
            cv::KeyPoint keyPointToShow(keyPointInfo.getX(), keyPointInfo.getY(), keyPointInfo.getScale());
            keyPointsToShowSecond.emplace_back(keyPointToShow);
        }
        std::vector<cv::DMatch> matches1to2;
        for (const auto& match: matchesKeypoints) {
            matches1to2.push_back(cv::DMatch(match.first, match.second, 0.01));
        }

        cv::Mat outputImage;
        cv::drawMatches(imageNoKeyPointFirst, keyPointsToShowFirst,
                        imageNoKeyPointSecond, keyPointsToShowSecond,
                        matches1to2,
                        outputImage);


        cv::imshow( "Showing matches", outputImage);
        cv::waitKey(0);
        return 0;
    }
}
