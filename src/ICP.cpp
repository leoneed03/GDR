//
// Created by leoneed on 1/9/21.
//

#include "ICP.h"
#include "ICPOdometry.h"
#include "pointCloud.h"

#include <pangolin/image/image_io.h>


namespace gdr {

    int loadDepth(pangolin::Image<unsigned short> &depth, const std::string &fileName, unsigned int width = 640,
                  unsigned int height = 480) {
        pangolin::TypedImage depthRaw =
                pangolin::LoadImage(fileName, pangolin::ImageFileTypePng);

        pangolin::Image<unsigned short> depthRaw16(
                (unsigned short *) depthRaw.ptr, depthRaw.w, depthRaw.h,
                depthRaw.w * sizeof(unsigned short));

        for (unsigned int i = 0; i < height; ++i) {
            for (unsigned int j = 0; j < width; ++j) {
                depth.RowPtr(i)[j] = depthRaw16(j, i) / 5;
            }
        }

        depthRaw.Dealloc();

        return 0;
    }

    int ProcessorICP::refineRelativePoseICP(const VertexCG &poseToBeTransformed,
                                            const VertexCG &poseDestination,
                                            Eigen::Matrix4d &initRelPosEstimation) {

//        cv::Mat depthImageToBeTransformed = cv::imread(poseToBeTransformed.pathToDimage, cv::IMREAD_ANYDEPTH);
//        cv::Mat projectedImageUmeyama = getProjectedPointCloud(poseToBeTransformed.pathToDimage, initRelPosEstimation, poseToBeTransformed.cameraRgbd);
//        cv::Mat depthImageDestination = cv::imread(poseDestination.pathToDimage, cv::IMREAD_ANYDEPTH);
//        //std::cout << "to be transformed \n"<< poseToBeTransformed.pathToDimage << std::endl;
//        //std::cout << "dest pos\n" << poseDestination.pathToDimage << std::endl;
//        cv::imshow("Before Transformation", depthImageToBeTransformed);
//        cv::imshow("got this (after transformation)", projectedImageUmeyama);
//        cv::imshow("Should get this (destination)", depthImageDestination);
//        cv::waitKey(0);
//        cv::destroyAllWindows();




        const auto &cameraRgbdOfToBeTransformed = poseToBeTransformed.cameraRgbd;
        ICPOdometry icpOdom(640, 480, cameraRgbdOfToBeTransformed.cx, cameraRgbdOfToBeTransformed.cy,
                            cameraRgbdOfToBeTransformed.fx, cameraRgbdOfToBeTransformed.fy);
        Sophus::SE3d T_wc_prev;
        Sophus::SE3d T_wc_curr;

        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        std::string dev(prop.name);
        //std::cout << "CUDA device used is " << dev << std::endl;

        int threads = 224;
        int blocks = 96;


        pangolin::ManagedImage<unsigned short> firstData(640, 480);
        pangolin::ManagedImage<unsigned short> secondData(640, 480);

        pangolin::Image<unsigned short> firstRaw(firstData.w, firstData.h,
                                                 firstData.pitch,
                                                 (unsigned short *) firstData.ptr);
        pangolin::Image<unsigned short> secondRaw(secondData.w, secondData.h,
                                                  secondData.pitch,
                                                  (unsigned short *) secondData.ptr);


        loadDepth(firstRaw, poseToBeTransformed.pathToDimage);
        loadDepth(secondRaw, poseDestination.pathToDimage);


        ///swap due to inverse order of images used by ICP
        std::swap(firstRaw, secondRaw);
        icpOdom.initICPModel(firstRaw.ptr);
        icpOdom.initICP(secondRaw.ptr);

        T_wc_prev = T_wc_curr;

//        Sophus::SE3d T_prev_curr = T_wc_prev.inverse() * T_wc_curr;
//        Sophus::SE3d relativeSE3_Rt(initRelPosEstimation.block<3, 3>(0, 0),
//                                    initRelPosEstimation.block<3, 1>(0, 3));
        Sophus::SE3d relativeSE3_Rt = Sophus::SE3d::fitToSE3(initRelPosEstimation);
//        relativeSE3_Rt = relativeSE3_Rt.inverse();

        Sophus::SE3d preICP_SE3 = relativeSE3_Rt;


        //std::cout << "Eigen Rotation Translation" << std::endl << initRelPosEstimation << std::endl;
//        //std::cout << "Sophus Translation" << relativeSE3_Rt.translation() << std::endl;

        //std::cout << "Sophus Rotation" << std::endl << relativeSE3_Rt.rotationMatrix() << std::endl;
        //std::cout << "Sophus Translation" << std::endl << relativeSE3_Rt.translation() << std::endl;
        //        = initRelPosEstimation;
        icpOdom.getIncrementalTransformation(relativeSE3_Rt, threads, blocks);


        //std::cout << "_______________________________________________" << std::endl;
//        std::cout << "Sophus Rotation AFTER" << std::endl << relativeSE3_Rt.rotationMatrix() << std::endl;
        //std::cout << "Sophus Translation AFTER" << std::endl << relativeSE3_Rt.translation() << std::endl;

        Eigen::Quaterniond sophusBeforeToEigen(preICP_SE3.rotationMatrix().matrix());
        Eigen::Quaterniond sophusAfterToEigen(relativeSE3_Rt.rotationMatrix().matrix());

        std::cout << "Rotation diff is " << std::endl << sophusAfterToEigen.angularDistance(sophusBeforeToEigen) << std::endl;
        std::cout << "Translation diff is " << std::endl << (preICP_SE3.translation() - relativeSE3_Rt.translation()).norm() << std::endl;
        //std::cout << "=========================================================================" << std::endl;


        ///finally refine the measurement
        initRelPosEstimation = relativeSE3_Rt.matrix();

//        //std::cout << "After all " << std::endl << initRelPosEstimation << std::endl;
////        //std::cout << "Total " << std::endl << initRelPosEstimation << std::endl;
//        cv::Mat depthImageToBeTransformed = cv::imread(poseToBeTransformed.pathToDimage, cv::IMREAD_ANYDEPTH);
//        cv::Mat projectedImageUmeyama = getProjectedPointCloud(poseToBeTransformed.pathToDimage, initRelPosEstimation,
//                                                               poseToBeTransformed.cameraRgbd);
//        cv::Mat depthImageDestination = cv::imread(poseDestination.pathToDimage, cv::IMREAD_ANYDEPTH);
//        //std::cout << "to be transformed \n" << poseToBeTransformed.pathToDimage << std::endl;
//        //std::cout << "dest pos\n" << poseDestination.pathToDimage << std::endl;
//        cv::imshow("Before Transformation", depthImageToBeTransformed);
//        cv::imshow("got this (after transformation)", projectedImageUmeyama);
//        cv::imshow("Should get this (destination)", depthImageDestination);
//        cv::waitKey(0);
//        cv::destroyAllWindows();

        return 0;
    }
}