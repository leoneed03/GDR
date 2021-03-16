//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "keyFeatures.h"
#include "printer.h"

#include "SiftGPU.h"


namespace gdr {

//    std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>> getKeypointsDescriptorsOneImage(
//            SiftGPU *sift,
//            const std::string &pathToTheImage) {
//
//        sift->RunSIFT(pathToTheImage.data());
//        int num1 = sift->GetFeatureNum();
//        std::vector<float> descriptors1(128 * num1);
//        std::vector<SiftGPU::SiftKeypoint> keys1(num1);
//        sift->GetFeatureVector(&keys1[0], &descriptors1[0]);
//        PRINT_PROGRESS(num1 << " -- total number of keypoints");
//        return {keys1, descriptors1};
//    }

//    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
//    getKeypointsDescriptorsAllImages(
//            SiftGPU *sift,
//            const std::string &pathToTheDirectory) {
//
//        std::vector<std::string> pathsToAllImages = readRgbData(pathToTheDirectory);
//        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keypointsAndDescriptorsAllImages;
//        keypointsAndDescriptorsAllImages.reserve(pathsToAllImages.size());
//        for (const auto &pathToTheImage: pathsToAllImages) {
//            keypointsAndDescriptorsAllImages.emplace_back(getKeypointsDescriptorsOneImage(sift, pathToTheImage));
//        }
//        return keypointsAndDescriptorsAllImages;
//    }

//    std::vector<std::pair<int, int>>
//    getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
//                                 const imageDescriptor &keysDescriptors2,
//                                 SiftMatchGPU *matcher) {
//
//        auto descriptors1 = keysDescriptors1.second;
//        auto descriptors2 = keysDescriptors2.second;
//
//        auto keys1 = keysDescriptors1.first;
//        auto keys2 = keysDescriptors2.first;
//
//        auto num1 = keys1.size();
//        auto num2 = keys2.size();
//
//        assert(num1 * 128 == descriptors1.size());
//        assert(num2 * 128 == descriptors2.size());
//
//
//        std::cout << "set descriptors" << std::endl;
//        matcher->SetDescriptors(0, num1, descriptors1.data()); //image 1
//        matcher->SetDescriptors(1, num2, descriptors2.data()); //image 2
//
//
//        std::vector<std::pair<int, int>> matchingKeypoints;
//        int (*match_buf)[2] = new int[num1][2];
//
//        std::cout << "get sift match" << std::endl;
//        int num_match = matcher->GetSiftMatch(num1, match_buf);
//        matchingKeypoints.reserve(num_match);
//
//        for (int i = 0; i < num_match; ++i) {
//            matchingKeypoints.emplace_back(match_buf[i][0], match_buf[i][1]);
//        }
//        delete[] match_buf;
//        return matchingKeypoints;
//    }
}
