//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>
#include <gtest/gtest.h>
#include <vector>

#include "CorrespondenceGraph.h"
#include "ImageDrawer.h"

#define epsilonD (std::numeric_limits<double>::epsilon())

// visualize points on different images
/*

TEST(testPointCloudCorrespondences, showOnePoint) {

    std::string pathToRGBImage = "../../data/plantDataset_19_3/depth/1305032354.109860.png";
//    std::string pathToRGBImage = "../../data/plantDataset_19_3/rgb/1305032354.093194.png";
    cv::Mat imageNoKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);

    int x = 50;
    int y = 200;
    cv::KeyPoint keyPointToShow(x, y, 50);

    cv::Mat imageWithKeyPoint = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);
    cv::drawKeypoints(imageNoKeyPoint, {keyPointToShow}, imageWithKeyPoint);


//        cv::namedWindow( );
    cv::imshow("Showing " + pathToRGBImage + " " + std::to_string(0), imageWithKeyPoint);
    cv::waitKey(0);

    cv::Mat imageRaw = cv::imread(pathToRGBImage, cv::IMREAD_COLOR);

    cv::Vec3b &color = imageRaw.at<cv::Vec3b>(cv::Point(x, y));
    color[0] = color[1] = color[2] = 255;
    cv::imshow("Showing x=50 y=200 at(x,y)", imageRaw);
    cv::waitKey(0);

}

TEST(testPointCloudCorrespondences, cloudProjectorShouldBeFilled) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    std::cout << "compute poses" << std::endl;
    correspondenceGraph.computeRelativePoses();

    auto &cloudProjector = correspondenceGraph.getCloudProjector();
//    gdr::ImageDrawer::showKeyPointOnImage(cloudProjector.getPoseByPoseNumber(0).getPathDImage(), pointsToShow[0].second, 0);

    for (int i = 0; i < cloudProjector.getPoseNumber(); ++i) {

        const auto &pointsToShow = cloudProjector.getKeyPointsIndicesAndInfoByPose(i);
        std::cout << "to show " << pointsToShow.size() << " on image " << i << std::endl;
        gdr::ImageDrawer::showKeyPointsOnImage(cloudProjector.getPoseByPoseNumber(i).getPathDImage(),
                                               pointsToShow,
                                               100,
                                               "../../data/tempImagesWIthKeyPointsCorrespondences",
                                               std::to_string(i) + ".png");
    }

}
*/
int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}