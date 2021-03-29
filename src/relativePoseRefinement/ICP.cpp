//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "relativePoseRefinement/ICP.h"
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

    bool ProcessorICP::refineRelativePose(const MatchableInfo &poseToBeTransformed,
                                          const MatchableInfo &poseDestination,
                                          SE3 &initTransformationSE3) {

        int height = poseToBeTransformed.getImagePixelHeight();
        int width = poseToBeTransformed.getImagePixelWidth();

        assert(height == poseDestination.getImagePixelHeight());
        assert(width == poseDestination.getImagePixelWidth());

        const auto &cameraRgbdOfToBeTransformed = poseToBeTransformed.getCameraRGB();
        ICPOdometry icpOdom(width, height,
                            cameraRgbdOfToBeTransformed.getCx(), cameraRgbdOfToBeTransformed.getCy(),
                            cameraRgbdOfToBeTransformed.getFx(), cameraRgbdOfToBeTransformed.getFy());

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

        loadDepth(imageICPModel, poseDestination.getPathImageD(), width, height);
        loadDepth(imageICP, poseToBeTransformed.getPathImageD(), width, height);

        icpOdom.initICPModel(imageICPModel.ptr);
        icpOdom.initICP(imageICP.ptr);

        Sophus::SE3d relativeSE3_Rt = initTransformationSE3.getSE3();

        icpOdom.getIncrementalTransformation(relativeSE3_Rt, threads, blocks);

        initTransformationSE3 = SE3(relativeSE3_Rt);

        return true;
    }
}