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

#include "poseGraph/VertexCG.h"
#include "SiftGPU.h"
#include "keyPoints/KeyPoint2D.h"

#include "ISiftModule.h"

namespace gdr {

    using imageDescriptor = std::pair<std::vector<KeyPoint2D>, std::vector<float>>;

    class SiftModuleGPU : public ISiftModule {

        std::unique_ptr<SiftMatchGPU> matcher;
        int maxSift = 4096;

    public:
        SiftModuleGPU();

        std::vector<std::pair<std::vector<KeyPoint2D>, std::vector<float>>>
        getKeypoints2DDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                           const std::vector<int> &numOfDevicesForDetectors = {0}) override;

        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>>
        getKeypointsDescriptorsAllImages(const std::vector<std::string> &pathsToImages,
                                         const std::vector<int> &numOfDevicesForDetectors = {0});

        std::vector<std::vector<Match>> findCorrespondences(const std::vector<VertexCG> &verticesToBeMatched,
                                                            const std::vector<int> &matchDevicesNumbers = {
                                                                    0}) override;

    private:


        std::vector<tbb::concurrent_vector<Match>> findCorrespondencesConcurrent(const std::vector<VertexCG> &verticesToBeMatched,
                                                                       const std::vector<int> &matchDevicesNumbers = {
                                                                               0});

        static std::vector<std::pair<int, int>>
        getNumbersOfMatchesKeypoints(const imageDescriptor &keysDescriptors1,
                                     const imageDescriptor &keysDescriptors2,
                                     SiftMatchGPU *matcher);

        void siftParseParams(SiftGPU *sift, std::vector<char *> &siftGpuArgs);


        /* L1-Root-normalize feature descriptors
         * @param descriptors -- matrix of N descriptors
         * each row of matrix represents one feature.
         */

        static
        Eigen::MatrixXf normalizeDescriptorsL1Root(const Eigen::MatrixXf &descriptors);

        static
        std::vector<float> normalizeDescriptorsL1Root(const std::vector<float> &descriptors);


        static void
        getKeypointsDescriptorsOneImage(SiftGPU *detectorSift,
                                        tbb::concurrent_queue<std::pair<std::string, int>> &pathsToImagesAndImageIndices,
                                        std::vector<std::pair<std::vector<SiftGPU::SiftKeypoint>, std::vector<float>>> &keyPointsAndDescriptorsByIndex,
                                        std::mutex &output,
                                        bool normalizeRootL1 = true);

        static void getNumbersOfMatchesOnePair(int &indexFrom,
                                               int &indexTo,
                                               const std::vector<VertexCG> &verticesToBeMatched,
                                               std::mutex &counterMutex,
                                               std::vector<tbb::concurrent_vector<Match>> &matches,
                                               SiftMatchGPU *matcher);
    };
}

#endif
