//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include "keyPoints/KeyPointsDepthDescriptor.h"

#include <vector>
#include <cassert>
#include <string>

namespace gdr {


    keyPointsDepthDescriptor::keyPointsDepthDescriptor(const std::vector<KeyPoint2DAndDepth> &newKeypointsKnownDepth,
                                                       const std::vector<float> &newDescriptorsKnownDepth,
                                                       const std::vector<double> &newDepths) :
            keypointsKnownDepth(newKeypointsKnownDepth),
            descriptorsKnownDepth(newDescriptorsKnownDepth),
            depths(newDepths) {

        assert(depths.size() == keypointsKnownDepth.size());
        assert(depths.size() * 128 == descriptorsKnownDepth.size());

        for (int pointIndex = 0; pointIndex < keypointsKnownDepth.size(); ++pointIndex) {
            keypointsKnownDepth[pointIndex].setDepth(depths[pointIndex]);
            assert(std::abs(keypointsKnownDepth[pointIndex].getDepth() - depths[pointIndex]) <
                   3 * std::numeric_limits<double>::epsilon());
        }
    }

    const std::vector<KeyPoint2DAndDepth> &keyPointsDepthDescriptor::getKeyPointsKnownDepth() const {
        return keypointsKnownDepth;
    }

    const std::vector<float> &keyPointsDepthDescriptor::getDescriptorsKnownDepth() const {
        return descriptorsKnownDepth;
    }

    const std::vector<double> &keyPointsDepthDescriptor::getDepths() const {
        return depths;
    }

    keyPointsDepthDescriptor keyPointsDepthDescriptor::filterKeypointsByKnownDepth(
            const std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>> &keypointAndDescriptor,
            const std::string &pathToDImage) {

        double depthCoefficient = 5000.0;
        const std::vector<KeyPoint2DAndDepth> &keypoints = keypointAndDescriptor.first;
        const std::vector<float> &descriptors = keypointAndDescriptor.second;
        std::vector<KeyPoint2DAndDepth> keypointsKnownDepth;
        std::vector<float> descriptorsKnownDepth;
        std::vector<double> depths;

        cv::Mat depthImage = cv::imread(pathToDImage, cv::IMREAD_ANYDEPTH);

        for (int i = 0; i < keypoints.size(); ++i) {
            int posInDescriptorVector = 128 * i;
            int maxDepthValue = 65536;
            auto coordY = keypoints[i].getY();
            auto coordX = keypoints[i].getX();

            assert(coordX >= 0 && coordX < depthImage.cols);
            assert(coordY >= 0 && coordY < depthImage.rows);
            int currentKeypointDepth = depthImage.at<ushort>(static_cast<ushort>(coordY),
                                                             static_cast<ushort>(coordX));

            if (currentKeypointDepth > 0) {
                assert(currentKeypointDepth < maxDepthValue);
                depths.push_back(currentKeypointDepth / depthCoefficient);
                keypointsKnownDepth.push_back(keypoints[i]);
                std::vector<float> currentDescriptors;
                for (int descriptorCounter = 0; descriptorCounter < 128; ++descriptorCounter) {
                    descriptorsKnownDepth.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                    currentDescriptors.push_back(descriptors[posInDescriptorVector + descriptorCounter]);
                }
            }
        }

        return keyPointsDepthDescriptor(keypointsKnownDepth, descriptorsKnownDepth, depths);

    }
}