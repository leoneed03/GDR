//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <thread>
#include <mutex>
#include "siftModule.h"
#include "printer.h"


namespace gdr {

    Eigen::MatrixXf SiftModule::normalizeDescriptorsL1Root(const Eigen::MatrixXf &descriptors) {
        Eigen::MatrixXf descriptors_normalized(descriptors.rows(),
                                               descriptors.cols());
        for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
            const float norm = descriptors.row(r).lpNorm<1>();
            descriptors_normalized.row(r) = descriptors.row(r) / norm;
            descriptors_normalized.row(r) =
                    descriptors_normalized.row(r).array().sqrt();
        }
        return descriptors_normalized;
    }

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

        std::cout << "matcher created by SiftModule Constructor" << std::endl;
        auto contextVerified = matcher->VerifyContextGL();
        if (contextVerified == 0) {
            std::cout << "_____________________________________________________matching context not verified"
                      << std::endl;
        }
//        assert(contextVerified != 0);
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
            //TODO: right args
            std::vector<std::string> siftGpuArgsStrings = {"-cuda", std::to_string(numOfDevicesForDetection[i]),
                                                           "-fo", "-1",
                                                           "-v", /*siftGpuFarg*/"1"};
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
                                     std::ref(output),
                                     true);
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
                                                std::mutex &output,
                                                bool normalizeRootL1) {

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

            // L1 Root normalization of secriptors:
            if (normalizeRootL1) {
                descriptors1 = normalizeDescriptorsL1Root(descriptors1);
            }

            assert(pathToImageAndIndex.second >= 0 &&
                   pathToImageAndIndex.second < keyPointsAndDescriptorsByIndex.size());
            keyPointsAndDescriptorsByIndex[pathToImageAndIndex.second] = {keys1, descriptors1};
        }
    }

    std::vector<tbb::concurrent_vector<Match>>
    SiftModule::findCorrespondences(const std::vector<VertexCG> &verticesToBeMatched,
                                    const std::vector<int> &matchDevicesNumbers) {

        std::vector<std::unique_ptr<SiftMatchGPU>> matchers;

        std::cout << "create matchers" << std::endl;
        for (int i = 0; i < matchDevicesNumbers.size(); ++i) {
            matchers.push_back(std::make_unique<SiftMatchGPU>(maxSift));
            auto contextVerified = matchers[i]->VerifyContextGL();
            assert(contextVerified != 0);
        }

        std::vector<tbb::concurrent_vector<Match>> matches(verticesToBeMatched.size());
        if (verticesToBeMatched.size() == 1 || verticesToBeMatched.empty()) {
            return matches;
        }

        int numberPoseFromLess = 0;
        int numberPoseToBigger = 1;
        int numberPoses = verticesToBeMatched.size();
        std::mutex counterMutex;

        std::vector<std::thread> threads(matchDevicesNumbers.size());
        for (int i = 0; i < threads.size(); ++i) {

            threads[i] = std::thread(getNumbersOfMatchesOnePair,
                                     std::ref(numberPoseFromLess), std::ref(numberPoseToBigger),
                                     std::ref(verticesToBeMatched),
                                     std::ref(counterMutex),
                                     std::ref(matches),
                                     matchers[i].get());
        }

        for (auto &thread: threads) {
            thread.join();
        }

        for (int i = 0; i < matches.size(); ++i) {
            std::cout << "matching from " << i << ": " << matches[i].size() << std::endl;
            assert(matches[i].size() == (matches.size() - i) - 1);
        }

        /*
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
        }*/
        return matches;
    };

    void SiftModule::getNumbersOfMatchesOnePair(int &indexFrom,
                                                int &indexTo,
                                                const std::vector<VertexCG> &verticesToBeMatched,
                                                std::mutex &counterMutex,
                                                std::vector<tbb::concurrent_vector<Match>> &matches,
                                                SiftMatchGPU *matcher) {
        while (true) {
            int localIndexFrom = -1;
            int localIndexTo = -1;

            counterMutex.lock();

            std::cout << indexFrom << " -[matching]-> " << indexTo << " of " << matches.size() << std::endl;
            assert(indexFrom < indexTo);

            if (indexTo >= matches.size()) {
                ++indexFrom;
                indexTo = indexFrom + 1;
            }
            if (indexFrom >= matches.size() || indexTo >= matches.size()) {
                counterMutex.unlock();
                return;
            }
            assert(indexFrom < indexTo);
            std::cout << "local indices are: " << indexFrom << " -[matching]-> " << indexTo << " of " << matches.size()
                      << "                " << matcher << std::endl;


            localIndexFrom = indexFrom;
            localIndexTo = indexTo;

            ++indexTo;
            counterMutex.unlock();

            std::vector<std::pair<int, int>> matchingNumbers = getNumbersOfMatchesKeypoints(
                    std::make_pair(verticesToBeMatched[localIndexFrom].keypoints,
                                   verticesToBeMatched[localIndexFrom].descriptors),
                    std::make_pair(verticesToBeMatched[localIndexTo].keypoints,
                                   verticesToBeMatched[localIndexTo].descriptors),
                    matcher);
            std::cout << "matched keypoint pairs " << matchingNumbers.size() << std::endl;
            matches[localIndexFrom].push_back({localIndexTo, matchingNumbers});

        }
    }

    std::vector<float> SiftModule::normalizeDescriptorsL1Root(const std::vector<float> &descriptorsToNormalize) {
        int descriptorLength = 128;

        std::vector<float> descriptors = descriptorsToNormalize;

        assert(descriptors.size() % descriptorLength == 0);
        int numberDescriptors = static_cast<int>(descriptors.size()) / 128;

        for (int descriptorNumber; descriptorNumber < numberDescriptors; ++descriptorNumber) {
            Eigen::Map<Eigen::VectorXf> descriptor(
                    descriptors.data() + descriptorNumber * descriptorLength,
                    descriptorLength);
            const float norm = descriptor.lpNorm<1>();
            descriptor = descriptor / norm;
            descriptor = descriptor.array().sqrt();
        }

        return descriptors;
    }
}
