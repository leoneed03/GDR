//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "ICPOdometry.h"

#include <pangolin/image/image_io.h>

#include "relativePoseRefinement/ICPCUDA.h"

namespace gdr {

    int loadDepthImage(pangolin::Image<unsigned short> &imageDepth,
                       const std::string &filename,
                       double depthDivider,
                       int width,
                       int height) {

        pangolin::TypedImage depthRawImage =
                pangolin::LoadImage(filename, pangolin::ImageFileTypePng);

        pangolin::Image<unsigned short> depthRaw16(
                (unsigned short *) depthRawImage.ptr, depthRawImage.w, depthRawImage.h,
                depthRawImage.w * sizeof(unsigned short));

        int depthDividerMm = static_cast<int>(depthDivider) / 1000;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                imageDepth.RowPtr(y)[x] = depthRaw16(x, y) / depthDividerMm;
            }
        }

        depthRawImage.Dealloc();

        return 0;
    }


    bool ICPCUDA::refineRelativePose(const MatchableInfo &poseToBeTransformedICP,
                                     const MatchableInfo &poseDestinationICPModel,
                                     const KeyPointMatches &keyPointMatches,
                                     SE3 &initTransformationSE3,
                                     int deviceIndex) {

        const CameraRGBD &cameraRgbdToBeTransformed = poseToBeTransformedICP.getCameraRGB();
        CameraIntrinsics cameraIntrinsicsToBeTransformed(cameraRgbdToBeTransformed.getFx(),
                                                         cameraRgbdToBeTransformed.getCx(),
                                                         cameraRgbdToBeTransformed.getFy(),
                                                         cameraRgbdToBeTransformed.getCy());

        const CameraRGBD &cameraRgbdDestination = poseDestinationICPModel.getCameraRGB();
        CameraIntrinsics cameraIntrinsicsDestination(cameraRgbdDestination.getFx(),
                                                     cameraRgbdDestination.getCx(),
                                                     cameraRgbdDestination.getFy(),
                                                     cameraRgbdDestination.getCy());

        int height = poseToBeTransformedICP.getImagePixelHeight();
        int width = poseToBeTransformedICP.getImagePixelWidth();

        assert(height == poseDestinationICPModel.getImagePixelHeight());
        assert(width == poseDestinationICPModel.getImagePixelWidth());

        const auto &cameraRgbdOfToBeTransformed = poseToBeTransformedICP.getCameraRGB();

        {
            std::unique_lock<std::mutex> deviceLockForICP(deviceCudaLock);
            ICPOdometry icpOdom(width, height);

            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, deviceIndex);
            std::string dev(prop.name);

            int threads = 224;
            int blocks = 96;

            pangolin::ManagedImage<unsigned short> firstData(width, height);
            pangolin::ManagedImage<unsigned short> secondData(width, height);

            pangolin::Image<unsigned short> imageICP(firstData.w, firstData.h,
                                                     firstData.pitch,
                                                     (unsigned short *) firstData.ptr);
            pangolin::Image<unsigned short> imageICPModel(secondData.w, secondData.h,
                                                          secondData.pitch,
                                                          (unsigned short *) secondData.ptr);

            loadDepthImage(imageICPModel, poseDestinationICPModel.getPathImageD(),
                           cameraRgbdDestination.getDepthPixelDivider(),
                           width, height);
            loadDepthImage(imageICP, poseToBeTransformedICP.getPathImageD(),
                           cameraRgbdOfToBeTransformed.getDepthPixelDivider(),
                           width, height);

            icpOdom.initICPModel(imageICPModel.ptr, cameraIntrinsicsDestination);
            icpOdom.initICP(imageICP.ptr, cameraIntrinsicsToBeTransformed);

            Sophus::SE3d relativeSE3_Rt = initTransformationSE3.getSE3();

            int iterationsLevel0 = 5;
            int iterationsLevel1 = 5;
            int iterationsLevel2 = 10;

            icpOdom.getIncrementalTransformation(relativeSE3_Rt,
                                                 true,
                                                 threads, blocks,
                                                 iterationsLevel0, iterationsLevel1, iterationsLevel2);

            initTransformationSE3 = SE3(relativeSE3_Rt);
        }

        return true;
    }
}