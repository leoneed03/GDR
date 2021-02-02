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
        cv::imshow( "Showing " + pathToRGBImage + " " + std::to_string(pointIndex), imageWithKeyPoint);
        cv::waitKey(0);
        return 0;
    }
}
