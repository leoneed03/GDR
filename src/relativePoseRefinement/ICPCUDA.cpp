//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseRefinement/ICPCUDA.h"
#include "ICPOdometry.h"

#include <pangolin/image/image_io.h>


namespace gdr {

    int loadDepth(pangolin::Image<unsigned short> &imageDepth,
                  const std::string &filename,
                  int width,
                  int height) {

        pangolin::TypedImage depthRawImage =
                pangolin::LoadImage(filename, pangolin::ImageFileTypePng);

        pangolin::Image<unsigned short> depthRaw16(
                (unsigned short *) depthRawImage.ptr, depthRawImage.w, depthRawImage.h,
                depthRawImage.w * sizeof(unsigned short));

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                imageDepth.RowPtr(y)[x] = depthRaw16(x, y) / 5;
            }
        }

        depthRawImage.Dealloc();

        return 0;
    }


    bool ICPCUDA::refineRelativePose(const MatchableInfo &poseToBeTransformedICP,
                                     const MatchableInfo &poseDestinationICPModel,
                                     SE3 &initTransformationSE3) {

        const CameraRGBD &cameraRgbdToBeTransformed = poseToBeTransformedICP.getCameraRGB();
        CameraIntrinsics cameraIntrinsicsToBeTransformed(cameraIntrinsicsToBeTransformed.getFx(),
                                                         cameraIntrinsicsToBeTransformed.getCx(),
                                                         cameraIntrinsicsToBeTransformed.getFy(),
                                                         cameraIntrinsicsToBeTransformed.getCy());

        const CameraRGBD &cameraRgbdDestination = poseDestinationICPModel.getCameraRGB();
        CameraIntrinsics cameraIntrinsicsDestination(cameraIntrinsicsDestination.getFx(),
                                                     cameraIntrinsicsDestination.getCx(),
                                                     cameraIntrinsicsDestination.getFy(),
                                                     cameraIntrinsicsDestination.getCy());

        int height = poseToBeTransformedICP.getImagePixelHeight();
        int width = poseToBeTransformedICP.getImagePixelWidth();

        assert(height == poseDestinationICPModel.getImagePixelHeight());
        assert(width == poseDestinationICPModel.getImagePixelWidth());

        const auto &cameraRgbdOfToBeTransformed = poseToBeTransformedICP.getCameraRGB();
        ICPOdometry icpOdom(width, height);

        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
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

        loadDepth(imageICPModel, poseDestinationICPModel.getPathImageD(), width, height);
        loadDepth(imageICP, poseToBeTransformedICP.getPathImageD(), width, height);

        icpOdom.initICPModel(imageICPModel.ptr, cameraIntrinsicsDestination);
        icpOdom.initICP(imageICP.ptr, cameraIntrinsicsToBeTransformed);

        Sophus::SE3d relativeSE3_Rt = initTransformationSE3.getSE3();

        int iterationsLevel0 = 10;
        int iterationsLevel1 = 5;
        int iterationsLevel2 = 4;

        icpOdom.getIncrementalTransformation(relativeSE3_Rt,
                                             true,
                                             threads, blocks,
                                             iterationsLevel0, iterationsLevel1, iterationsLevel2);

        initTransformationSE3 = SE3(relativeSE3_Rt);

        return true;
    }
}