//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_SIFT_H
#define GDR_SIFT_H

#include <vector>
#include <memory>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <mutex>

#include <Eigen/Eigen>

#include "SiftGPU.h"

#include "keyPoints/KeyPoint2DAndDepth.h"

#include "FeatureDetectorMatcher.h"
#include "keyPointDetectionAndMatching/ImageRetriever.h"

namespace gdr {

    using imageDescriptor = std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>;

    class SiftModuleGPU : public FeatureDetectorMatcher {

    public:
        enum class PrintDebug {
            NOTHING, EVERYTHING
        };

    private:
        PrintDebug whatToPrint = PrintDebug::NOTHING;
        int maxSift = 4096;

    public:

        std::vector<std::pair<std::vector<KeyPoint2DAndDepth>, std::vector<float>>>
        getKeypoints2DDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                           const std::vector<int> &numOfDevicesForDetectors) override;

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
        getKeypointsDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                         const std::vector<int> &numOfDevicesForDetectors);

        std::vector<std::vector<Match>>
        findCorrespondences(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                            ImageRetriever &imageRetriever,
                            const std::vector<int> &matchDevicesNumbers) override;

    private:


        std::vector<tbb::concurrent_vector<Match>>
        findCorrespondencesConcurrent(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                                      ImageRetriever &imageRetriever,
                                      const std::vector<int> &matchDevicesNumbers);

        static std::vector<std::pair<int, int>>
        getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                                     const imageDescriptor &keysDescriptors2,
                                     SiftMatchGPU *matcher,
                                     std::vector<int[2]> &matchesToPut);

        void siftParseParams(SiftGPU *sift, std::vector<char *> &siftGpuArgs);


        static
        std::vector<float> normalizeDescriptorsL1Root(const std::vector<float> &descriptors);


        static void
        getKeypointsDescriptorsOneImage(SiftGPU *detectorSift,
                                        tbb::concurrent_queue<std::pair<std::string, int>> &pathsToImagesAndImageIndices,
                                        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> &keyPointsAndDescriptorsByIndex,
                                        std::mutex &output,
                                        bool normalizeRootL1 = true);

        static void getNumbersOfMatchesOnePair(const std::vector<KeyPointsDescriptors> &verticesToBeMatched,
                                               std::vector<tbb::concurrent_vector<Match>> &matches,
                                               SiftMatchGPU *matcher,
                                               ImageRetriever &imageRetriever);
    };
}

#endif
