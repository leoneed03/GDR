//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <thread>
#include <mutex>

#include "keyPointDetectionAndMatching/KeyPointsAndDescriptors.h"
#include "keyPointDetectionAndMatching/SiftModuleGPU.h"

namespace gdr {

    void SiftModuleGPU::siftParseParams(SiftGPU *sift, std::vector<char *> &siftGpuArgs) {
        sift->ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
    }

    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
    SiftModuleGPU::getKeypointsDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                                    const std::vector<int> &numOfDevicesForDetection) {

        std::vector<std::unique_ptr<SiftGPU>> detectorsSift;
        for (const auto &device: numOfDevicesForDetection) {
            detectorsSift.push_back(std::move(std::make_unique<SiftGPU>()));
        }

        for (int i = 0; i < numOfDevicesForDetection.size(); ++i) {

            const auto &detector = detectorsSift[i];

            bool printDebugSymbols = (whatToPrint == PrintDebug::EVERYTHING);
            std::string siftGpuFarg = std::to_string(printDebugSymbols);

            std::vector<std::string> siftGpuArgsStrings = {"-cuda",
                                                           std::to_string(numOfDevicesForDetection[i]),
                                                           "-fo", "-1",
                                                           "-v", siftGpuFarg};
            std::vector<char *> siftGpuArgs;

            for (auto &stringArg: siftGpuArgsStrings) {
                siftGpuArgs.push_back((char *) stringArg.data());
            }

            detector->ParseParam(siftGpuArgs.size(), siftGpuArgs.data());

            auto contextVerified = detector->VerifyContextGL();
            assert(contextVerified != 0);
        }

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> keypointsAndDescriptorsAllImages(
                pathsToImages.size());

        tbb::concurrent_queue<std::pair<std::string, int>> pathsToImagesAndImageIndices;
        for (int i = 0; i < keypointsAndDescriptorsAllImages.size(); ++i) {
            pathsToImagesAndImageIndices.push({pathsToImages[i], i});
        }

        std::vector<std::thread> threads(numOfDevicesForDetection.size());

        std::mutex output;
        for (int i = 0; i < numOfDevicesForDetection.size(); ++i) {
            threads[i] = std::thread(SiftModuleGPU::getKeypointsDescriptorsOneImage,
                                     detectorsSift[i].get(),
                                     std::ref(pathsToImagesAndImageIndices),
                                     std::ref(keypointsAndDescriptorsAllImages),
                                     std::ref(output),
                                     true);
        }

        for (auto &thread: threads) {
            thread.join();
        }

        return keypointsAndDescriptorsAllImages;
    }

    void
    SiftModuleGPU::getKeypointsDescriptorsOneImage(SiftGPU *detectorSift,
                                                   tbb::concurrent_queue<std::pair<std::string, int>> &pathsToImagesAndImageIndices,
                                                   std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> &keyPointsAndDescriptorsByIndex,
                                                   std::mutex &output,
                                                   bool normalizeRootL1) {

        std::pair<std::string, int> pathToImageAndIndex;

        while (pathsToImagesAndImageIndices.try_pop(pathToImageAndIndex)) {
            const auto &pathToTheImage = pathToImageAndIndex.first;

            detectorSift->RunSIFT(pathToTheImage.data());
            int num1 = detectorSift->GetFeatureNum();
            std::vector<float> descriptors1(128 * num1);
            std::vector<SiftGPU::SiftKeypoint> keys1(num1);
            detectorSift->GetFeatureVector(keys1.data(), descriptors1.data());

            if (normalizeRootL1) {
                descriptors1 = normalizeDescriptorsL1Root(descriptors1);
            }

            assert(pathToImageAndIndex.second >= 0 &&
                   pathToImageAndIndex.second < keyPointsAndDescriptorsByIndex.size());
            keyPointsAndDescriptorsByIndex[pathToImageAndIndex.second] = {keys1, descriptors1};
        }
    }


    std::vector<std::pair<int, int>>
    SiftModuleGPU::getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                                                const imageDescriptor &keysDescriptors2,
                                                SiftMatchGPU *matcher,
                                                std::vector<int[2]> &matchesToPut) {

        auto &descriptors1 = keysDescriptors1.second;
        auto &descriptors2 = keysDescriptors2.second;

        auto &keys1 = keysDescriptors1.first;
        auto &keys2 = keysDescriptors2.first;

        auto num1 = keys1.size();
        auto num2 = keys2.size();

        assert(num1 * 128 == descriptors1.size());
        assert(num2 * 128 == descriptors2.size());

        matcher->SetDescriptors(0, num1, descriptors1.data());
        matcher->SetDescriptors(1, num2, descriptors2.data());

        std::vector<std::pair<int, int>> matchingKeypoints;

        int (*match_buf)[2] = matchesToPut.data();
        int num_match = matcher->GetSiftMatch(num1, match_buf);
        matchingKeypoints.reserve(num_match);

        for (int i = 0; i < num_match; ++i) {
            matchingKeypoints.emplace_back(match_buf[i][0], match_buf[i][1]);
        }

        return matchingKeypoints;
    }

    std::vector<std::vector<Match>>
    SiftModuleGPU::findCorrespondences(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                                       ImageRetriever &imageRetriever,
                                       const std::vector<int> &matchDevicesNumbers) {

        auto matches = findCorrespondencesConcurrent(verticesToBeMatched,
                                                     imageRetriever,
                                                     matchDevicesNumbers);
        std::vector<std::vector<Match>> resultMatches(matches.size());

        for (int i = 0; i < matches.size(); ++i) {
            for (const auto &match: matches[i]) {
                resultMatches[i].emplace_back(match);
            }
        }

        return resultMatches;
    }

    std::vector<tbb::concurrent_vector<Match>>
    SiftModuleGPU::findCorrespondencesConcurrent(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                                                 ImageRetriever &imageRetriever,
                                                 const std::vector<int> &matchDevicesNumbers) {

        std::vector<std::unique_ptr<SiftMatchGPU>> matchers;

        for (int i = 0; i < matchDevicesNumbers.size(); ++i) {
            matchers.push_back(std::move(std::make_unique<SiftMatchGPU>(maxSift)));
            matchers[i]->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA + matchDevicesNumbers[i]);

            auto contextVerified = matchers[i]->VerifyContextGL();
            assert(contextVerified != 0);
        }

        std::vector<tbb::concurrent_vector<Match>> matches(verticesToBeMatched.size());
        if (verticesToBeMatched.size() == 1 || verticesToBeMatched.empty()) {
            return matches;
        }

        std::vector<std::thread> threads(matchDevicesNumbers.size());
        for (int i = 0; i < threads.size(); ++i) {

            threads[i] = std::thread(getNumbersOfMatchesOnePair,
                                     std::ref(verticesToBeMatched),
                                     std::ref(matches),
                                     matchers[i].get(),
                                     std::ref(imageRetriever));
        }

        for (auto &thread: threads) {
            thread.join();
        }

        for (int i = 0; i < matches.size(); ++i) {
            assert(matches[i].size() == (matches.size() - i) - 1);
        }

        return matches;
    }

    void SiftModuleGPU::getNumbersOfMatchesOnePair(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                                                   std::vector<tbb::concurrent_vector<Match>> &matches,
                                                   SiftMatchGPU *matcher,
                                                   ImageRetriever &imageRetriever) {

        std::vector<int[2]> matchesToPut;

        while (true) {

            std::pair<int, int> indexFromLessAndToBiggerFromImageRetriever;
            bool existsPairToMatch = imageRetriever.tryGetSimilarImagesPair(
                    indexFromLessAndToBiggerFromImageRetriever);

            if (!existsPairToMatch) {
                return;
            }

            int localIndexFromLess = indexFromLessAndToBiggerFromImageRetriever.first;
            int localIndexToBigger = indexFromLessAndToBiggerFromImageRetriever.second;

            assert(localIndexFromLess < localIndexToBigger);
            assert(localIndexFromLess >= 0 && localIndexToBigger < verticesToBeMatched.size());
            assert(localIndexFromLess < localIndexToBigger);

            int sizeFrom = verticesToBeMatched[localIndexFromLess].getKeyPoints().size();
            int sizeTo = verticesToBeMatched[localIndexToBigger].getKeyPoints().size();

            int sizeToBeMatchedMax = std::min(sizeFrom, sizeTo);

            if (sizeToBeMatchedMax > matchesToPut.size()) {

                std::vector<int[2]> matchesToPutToSwap(sizeToBeMatchedMax);
                std::swap(matchesToPut, matchesToPutToSwap);
            }

            std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                    std::make_pair(verticesToBeMatched[localIndexFromLess].getKeyPoints(),
                                   verticesToBeMatched[localIndexFromLess].getDescriptors()),
                    std::make_pair(verticesToBeMatched[localIndexToBigger].getKeyPoints(),
                                   verticesToBeMatched[localIndexToBigger].getDescriptors()),
                    matcher,
                    matchesToPut);

            matches[localIndexFromLess].emplace_back(
                    std::move(gdr::Match(localIndexToBigger, std::move(matchingNumbers))));
        }
    }

    std::vector<float> SiftModuleGPU::normalizeDescriptorsL1Root(const std::vector<float> &descriptorsToNormalize) {
        int descriptorLength = 128;

        std::vector<float> descriptors = descriptorsToNormalize;

        assert(descriptors.size() % descriptorLength == 0);
        int numberDescriptors = static_cast<int>(descriptors.size()) / 128;

        for (int descriptorNumber; descriptorNumber < numberDescriptors; ++descriptorNumber) {
            Eigen::Map<Eigen::VectorXf> descriptor(
                    descriptors.data() + descriptorNumber * descriptorLength,
                    descriptorLength);

            const double norm = descriptor.lpNorm<1>();

            descriptor = descriptor / norm;
            descriptor = descriptor.array().sqrt();
        }

        return descriptors;
    }

    std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
    SiftModuleGPU::getKeypoints2DDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                                      const std::vector<int> &numOfDevicesForDetectors) {

        int descriptorLength = 128;
        auto keyPointsDescriptorsSiftGPU = getKeypointsDescriptorsAllImages(pathsToImages, numOfDevicesForDetectors);

        std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
                keyPointsAndDescriptorsAllImages(keyPointsDescriptorsSiftGPU.size());


        for (int imageNumber = 0; imageNumber < keyPointsDescriptorsSiftGPU.size(); ++imageNumber) {
            const auto &keyPointsAndDescriptorsOneImage = keyPointsDescriptorsSiftGPU[imageNumber];
            assert(keyPointsAndDescriptorsOneImage.second.size() ==
                   descriptorLength * keyPointsAndDescriptorsOneImage.first.size());

            for (const auto &siftKeyPoint: keyPointsAndDescriptorsOneImage.first) {
                keyPointsAndDescriptorsAllImages[imageNumber]
                        .first.emplace_back(KeyPoint2DAndDepth(siftKeyPoint.x, siftKeyPoint.y,
                                                               siftKeyPoint.s, siftKeyPoint.o));
            }

            keyPointsAndDescriptorsAllImages[imageNumber].second = keyPointsDescriptorsSiftGPU[imageNumber].second;

            assert(keyPointsAndDescriptorsAllImages[imageNumber].second.size() ==
                   keyPointsAndDescriptorsAllImages[imageNumber].first.size() * descriptorLength);
        }
        return keyPointsAndDescriptorsAllImages;
    }
}
