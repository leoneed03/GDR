//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <thread>
#include <mutex>
#include "siftModule.h"
#include "printer.h"


namespace gdr {

    void SiftModule::siftParseParams(SiftGPU *sift, std::vector<char *> &siftGpuArgs) {
        sift->ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
    }

    SiftModule::SiftModule() {

        std::string siftGpuFarg = std::to_string(SIFTGPU_ARG_V);
        std::vector<std::string> siftGpuArgsStrings = {"-cuda", "0", "-fo", "-1", "-v", siftGpuFarg};
        std::vector<char *> siftGpuArgs;

        for (auto &stringArg: siftGpuArgsStrings) {
            siftGpuArgs.push_back(stringArg.data());
        }
        matcher = std::make_unique<SiftMatchGPU>(SiftMatchGPU(maxSift));

        auto contextVerified = matcher->VerifyContextGL();
        if (contextVerified == 0) {
            std::cout << "_____________________________________________________matching context not verified"
                      << std::endl;
        }
        assert(contextVerified != 0);
    }

    std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
    SiftModule::getKeypointsDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                                 const std::vector<int> &numOfDevicesForDetection) {

        std::vector<std::unique_ptr<SiftGPU>> detectorsSift;
        for (const auto &device: numOfDevicesForDetection) {
            detectorsSift.push_back(std::make_unique<SiftGPU>());
        }

        for (int i = 0; i < numOfDevicesForDetection.size(); ++i) {
            std::string siftGpuFarg = std::to_string(SIFTGPU_ARG_V);
            const auto &detector = detectorsSift[i];
            std::vector<std::string> siftGpuArgsStrings = {"-cuda", std::to_string(numOfDevicesForDetection[i]),
                                                           "-fo", "-1",
                                                           "-v", siftGpuFarg};
            std::vector<char *> siftGpuArgs;

            for (auto &stringArg: siftGpuArgsStrings) {
                siftGpuArgs.push_back(stringArg.data());
            }
            detector->ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
            std::cout << "create context GL" << std::endl;
            int contextCreated = detector->CreateContextGL();
            if (contextCreated != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
                std::cout << "_____________________________________________________context not created"
                          << std::endl;
            }
            auto contextVerified = detector->VerifyContextGL();
            if (contextVerified == 0) {
                std::cout << "_____________________________________________________detection context not verified"
                          << std::endl;
            }
            assert(contextCreated == SiftGPU::SIFTGPU_FULL_SUPPORTED);
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
            threads[i] = std::thread(SiftModule::getKeypointsDescriptorsOneImage,
                                     detectorsSift[i].get(),
                                     std::ref(pathsToImagesAndImageIndices),
                                     std::ref(keypointsAndDescriptorsAllImages),
                                     std::ref(output));
        }

        for (auto &thread: threads) {
            thread.join();
        }

        return keypointsAndDescriptorsAllImages;
    }

    void
    SiftModule::getKeypointsDescriptorsOneImage(SiftGPU *detectorSift,
                                                tbb::concurrent_queue<std::pair<std::string, int>> &pathsToImagesAndImageIndices,
                                                std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> &keyPointsAndDescriptorsByIndex,
                                                std::mutex &output) {

        std::pair<std::string, int> pathToImageAndIndex;
        while (pathsToImagesAndImageIndices.try_pop(pathToImageAndIndex)) {
            const auto &pathToTheImage = pathToImageAndIndex.first;
            output.lock();
            std::cout << "image: " << pathToTheImage << std::endl;
            output.unlock();
            detectorSift->RunSIFT(pathToTheImage.data());
            int num1 = detectorSift->GetFeatureNum();
            std::vector<float> descriptors1(128 * num1);
            std::vector<SiftGPU::SiftKeypoint> keys1(num1);
            detectorSift->GetFeatureVector(keys1.data(), descriptors1.data());
            assert(pathToImageAndIndex.second >= 0 &&
                   pathToImageAndIndex.second < keyPointsAndDescriptorsByIndex.size());
            keyPointsAndDescriptorsByIndex[pathToImageAndIndex.second] = {keys1, descriptors1};
        }
    }

    std::vector<std::vector<Match>> SiftModule::findCorrespondences(const std::vector<VertexCG> &verticesToBeMatched) {

        std::vector<std::vector<Match>> matches(verticesToBeMatched.size());

        for (int i = 0; i < verticesToBeMatched.size(); ++i) {
            for (int j = i + 1; j < verticesToBeMatched.size(); ++j) {

                std::cout << "matching images " << i << " and " << j << std::endl;
                std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                        std::make_pair(verticesToBeMatched[i].keypoints, verticesToBeMatched[i].descriptors),
                        std::make_pair(verticesToBeMatched[j].keypoints, verticesToBeMatched[j].descriptors),
                        matcher.get());
                PRINT_PROGRESS("total matches " << matchingNumbers.size() << std::endl);
                matches[i].push_back({j, matchingNumbers});
            }
        }
        return matches;

    };
}
