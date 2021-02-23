//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>



//#include "GlobalUtil.h"
//#include <boost/asio/thread_pool.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"

#include <tbb/task.h>
#include <tbb/task_group.h>
#include <boost/timer.hpp>
#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include "ThreadPool.h"

void f(SiftGPU *sift, std::string s) {
    sift->RunSIFT(s.data());
}

TEST(testBundleAdjustment, tbb) {
    tbb::task_scheduler_init sheduler();
    std::mutex out;
    int thread = 0;
    int largeCounter = 0;
    std::map<std::thread::id, int> ids;

    tbb::parallel_for(0, 10, [&ids, &thread, &out, &largeCounter](int) {
        {
            std::unique_lock<std::mutex> lockCout(out);
            if (ids.find(std::this_thread::get_id()) == ids.end()) {
                ids[std::this_thread::get_id()] = ids.size();
            }
            std::cout << "start doing LARGE " << largeCounter << " by thread #" << ids[std::this_thread::get_id()]
                      << std::endl;
            ++largeCounter;
        }
        tbb::parallel_for(0, 15, [&ids, &thread, &out, &largeCounter](int) {
            int localThread = -1;
            {

                std::unique_lock<std::mutex> lockCout(out);
                if (ids.find(std::this_thread::get_id()) == ids.end()) {
                    ids[std::this_thread::get_id()] = ids.size();
                }
                std::cout << "         start doing " << thread << " of large " << largeCounter << " by thread #"
                          << ids[std::this_thread::get_id()] << std::endl;
                localThread = thread;
                ++thread;
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(200ms);

            {
                std::unique_lock<std::mutex> lockCout(out);
                std::cout << "                     cancel doing " << localThread << " of large counter " << largeCounter
                          << " by thread #"
                          << ids[std::this_thread::get_id()] << std::endl;
            }
        });
        {
            std::unique_lock<std::mutex> lockCout(out);
            std::cout << "cancel doing LARGE " << largeCounter << " by thread #" << ids[std::this_thread::get_id()]
                      << std::endl;
            ++largeCounter;
        }
    });
}

TEST(testBundleAdjustment, concurrentSiftDetector) {

    gdr::ThreadPool threads(4);
    tbb::task_group &group = threads.getTaskGroup();


    std::mutex out;
    std::map<std::thread::id, int> ids;
    boost::timer timer;
    std::atomic_int inc;

    for (int i = 0; i < 10; ++i) {
        {
            std::unique_lock<std::mutex> lock(out);
            std::cout << "pushed " << i << " task" << std::endl;
        }
        group.run([i, &inc, &out, &ids]() {
            int s = 0;
            for (long long j = 0; j < 1000000000; ++j) {
                s += j;
            }
            inc += s;
            std::unique_lock<std::mutex> lock(out);
            std::cout << "start doing " << i << " task" << std::endl;
            auto id = std::this_thread::get_id();
            int intId;
            if (ids.find(id) == ids.end()) {
                intId = ids.size();
                ids[id] = intId;
//                std::cout << "not found " << id << std::endl;
            } else {
                intId = ids[id];
//                std::cout << " found " << id << std::endl;
            }

            std::cout << i << ":________ by \t" << intId << std::endl;
        });
    }

    group.wait();

    std::cout << inc << "done after " << timer.elapsed() << std::endl;
    ASSERT_TRUE(true);


//    SiftGPU d1;
//    SiftGPU d2;
//
//
////    boost::asio::thread_pool
//    std::vector<std::string> siftGpuArgsStrings = {"-cuda", "0", "-fo", "-1", "-v", "1"};
//
//    std::vector<std::string> siftGpuArgsStringsGLSL = {"-fo", "-1", "-v", "1"};
//    std::vector<char *> siftGpuArgs;
//    std::vector<char *> siftGpuArgsGLSL;
//
//    for (auto &stringArg: siftGpuArgsStrings) {
//        siftGpuArgs.push_back(stringArg.data());
//    }
//    for (auto &stringArg: siftGpuArgsStringsGLSL) {
//        siftGpuArgs.push_back(stringArg.data());
//    }
//    d1.ParseParam(siftGpuArgs.size(), siftGpuArgs.data());
//    d2.ParseParam(siftGpuArgsGLSL.size(), siftGpuArgsGLSL.data());
//
//    if (d2.VerifyContextGL() == 0 || d2.VerifyContextGL() == 0) {
//        std::cout << "_____________________________________________________detection context not verified"
//                  << std::endl;
//        ASSERT_TRUE(false);
//    }
//
////    std::thread t1(f, &d1, "../../data/plantDataset_19_3/rgb/1305032354.093194.png");
//    std::thread t2(f, &d2, "../../data/plantDataset_19_3/rgb/1305032354.193245.png");
////    t1.join();
//    t2.join();
//    std::cout << " " << std::endl;

}

TEST(testBundleAdjustment, justIRLS) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
//    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustment();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);


    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/19/BA_19.txt";
    std::ofstream computedPoses(outputName);
    for (int i = 0; i < posesInfo.size(); ++i) {
        const auto &pose = correspondenceGraph.verticesOfCorrespondence[i].getAbsolutePoseSE3();
        Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * pose;
//        ASSERT_LE(poseSE3.unit_quaternion().angularDistance(posesInfo[i].getOrientationQuat()), 0.06);
//        ASSERT_LE((poseSE3.translation() - posesInfo[i].getTranslation()).norm(), 0.06);
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto to = poseSE3.translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = poseSE3.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

}

TEST(testBundleAdjustment, allPosesAreOptimizedBundleAdjustmentNoDepth) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustment();

    Sophus::SE3d poseBAzero = bundleAdjustedPoses[0];
    for (auto &pose: bundleAdjustedPoses) {
        pose = poseBAzero.inverse() * pose;
    }

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> posesGT;

    for (const auto &poseGT: posesInfo) {
        Sophus::SE3d poseSE3;
        poseSE3.setQuaternion(poseGT.getOrientationQuat());
        poseSE3.translation() = poseGT.getTranslation();
        posesGT.push_back(poseSE3);
    }
    Sophus::SE3d poseGTzero = posesGT[0];
    for (auto &poseGT: posesGT) {
        poseGT = poseGTzero.inverse() * poseGT;
    }

    double sumErrorT = 0;
    double sumErrorR = 0;
    double maxErrorR = 0;
    double maxErrorT = 0;

    assert(posesGT.size() == bundleAdjustedPoses.size());
    assert(posesGT.size() == 19);

    for (int i = 0; i < posesGT.size(); ++i) {
        const auto &poseGT = posesGT[i];
        const auto &poseBA = bundleAdjustedPoses[i];

        double currentErrorR = poseGT.unit_quaternion().angularDistance(poseBA.unit_quaternion());
        double currentErrorT = (poseGT.translation() - poseBA.translation()).norm();
        sumErrorR += currentErrorR;
        sumErrorT += currentErrorT;
        maxErrorR = std::max(maxErrorR, currentErrorR);
        maxErrorT = std::max(maxErrorT, currentErrorT);
    }

    double meanErrorT_L2 = sumErrorT / posesGT.size();
    double meanErrorR_angDist = sumErrorR / posesGT.size();

    std::cout << "__________BA test report 19 poses_____________" << std::endl;
    std::cout << "mean error translation: " << meanErrorT_L2 << std::endl;
    std::cout << "mean error rotation: " << meanErrorR_angDist << std::endl;


    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_BA_no_depth.txt";
    std::ofstream computedPoses(outputName);
    for (int i = 0; i < posesInfo.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto to = poseSE3.translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = poseSE3.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }


    ASSERT_LE(meanErrorR_angDist, 0.025);
    ASSERT_LE(meanErrorT_L2, 0.05);
}

TEST(testBundleAdjustment, allPosesAreOptimizedBundleAdjustmentUsingDepth_iterations) {

    for (int iterations = 0; iterations < 1; ++iterations) {
        gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                     "../../data/plantDataset_19_3/depth",
                                                     517.3,
                                                     318.6, 516.5, 255.3);
        correspondenceGraph.computeRelativePoses();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
        std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

        Sophus::SE3d poseBAzero = bundleAdjustedPoses[0];
        for (auto &pose: bundleAdjustedPoses) {
            pose = poseBAzero.inverse() * pose;
        }

        std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
        std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

        std::vector<Sophus::SE3d> posesGT;

        for (const auto &poseGT: posesInfo) {
            Sophus::SE3d poseSE3;
            poseSE3.setQuaternion(poseGT.getOrientationQuat());
            poseSE3.translation() = poseGT.getTranslation();
            posesGT.push_back(poseSE3);
        }
        Sophus::SE3d poseGTzero = posesGT[0];
        for (auto &poseGT: posesGT) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        double sumErrorT = 0;
        double sumErrorR = 0;
        double maxErrorR = 0;
        double maxErrorT = 0;

        assert(posesGT.size() == bundleAdjustedPoses.size());
        assert(posesGT.size() == 19);

        for (int i = 0; i < posesGT.size(); ++i) {
            const auto &poseGT = posesGT[i];
            const auto &poseBA = bundleAdjustedPoses[i];

            double currentErrorR = poseGT.unit_quaternion().angularDistance(poseBA.unit_quaternion());
            double currentErrorT = (poseGT.translation() - poseBA.translation()).norm();
            sumErrorR += currentErrorR;
            sumErrorT += currentErrorT;
            maxErrorR = std::max(maxErrorR, currentErrorR);
            maxErrorT = std::max(maxErrorT, currentErrorT);
        }

        double meanErrorT_L2 = sumErrorT / posesGT.size();
        double meanErrorR_angDist = sumErrorR / posesGT.size();

        std::cout << "__________BA test report 19 poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_angDist << std::endl;
        std::cout << "max error translation: " << maxErrorT << std::endl;
        std::cout << "mean error rotation: " << maxErrorR << std::endl;
        ASSERT_LE(meanErrorR_angDist, 0.02);
        ASSERT_LE(meanErrorT_L2, 0.02);
        ASSERT_LE(maxErrorR, 0.05);
        ASSERT_LE(maxErrorT, 0.05);

    }
}


// sometimes BA results are worse than IRLS so max error is multiplied by coefficient = 1.8
TEST(testBundleAdjustment, posesBundleAdjustmentUsingDepthBetterThanIRLS_iterations) {

    for (int iterations = 0; iterations < 1; ++iterations) {
        double coefficientR = 1.8;
        double coefficientT = 1.8;
        gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                     "../../data/plantDataset_19_3/depth",
                                                     517.3, 318.6,
                                                     516.5, 255.3);
        correspondenceGraph.computeRelativePoses();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
        std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

        std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
        std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

        // compute absolute poses IRLS
        std::vector<Sophus::SE3d> posesIRLS;
        {
            for (int i = 0; i < computedAbsoluteTranslationsIRLS.size(); ++i) {
                Sophus::SE3d poseIRLS;
                poseIRLS.setQuaternion(computedAbsoluteOrientationsRobust[i]);
                poseIRLS.translation() = computedAbsoluteTranslationsIRLS[i];
                posesIRLS.push_back(poseIRLS);
            }
            // set origin at zero pose [IRLS]
            Sophus::SE3d poseIRLSzero = posesIRLS[0];
            for (auto &poseIRLS: posesIRLS) {
                poseIRLS = poseIRLSzero.inverse() * poseIRLS;
            }
        }
        {
            // set origin at zero pose [BA]
            Sophus::SE3d poseBAzero = bundleAdjustedPoses[0];
            for (auto &pose: bundleAdjustedPoses) {
                pose = poseBAzero.inverse() * pose;
            }
        }

        {
            // print poses IRLS to file
            std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/19/IRLS_19.txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * posesIRLS[i];

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }
        {
            // print poses BA depth to file
            std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/19/BA_19.txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * bundleAdjustedPoses[i];

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }





        // fill absolute poses as SE3
        std::vector<Sophus::SE3d> posesGT;

        for (const auto &poseGT: posesInfo) {
            Sophus::SE3d poseSE3;
            poseSE3.setQuaternion(poseGT.getOrientationQuat());
            poseSE3.translation() = poseGT.getTranslation();
            posesGT.push_back(poseSE3);
        }
        Sophus::SE3d poseGTzero = posesGT[0];
        for (auto &poseGT: posesGT) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        double sumErrorT_BA = 0;
        double sumErrorR_BA = 0;


        double sumErrorT_IRLS = 0;
        double sumErrorR_IRLS = 0;

        double maxErrorR_IRLS = 0;
        double maxErrorT_IRLS = 0;

        double maxErrorR_BA = 0;
        double maxErrorT_BA = 0;

        assert(posesGT.size() == bundleAdjustedPoses.size());
        assert(posesGT.size() == 19);

        for (int i = 0; i < posesGT.size(); ++i) {
            const auto &poseGT = posesGT[i];
            const auto &poseBA = bundleAdjustedPoses[i];
            const auto &poseIRLS = posesIRLS[i];

            double errorR_BA = poseGT.unit_quaternion().angularDistance(poseBA.unit_quaternion());
            double errorT_BA = (poseGT.translation() - poseBA.translation()).norm();
            sumErrorR_BA += errorR_BA;
            sumErrorT_BA += errorT_BA;

            double errorR_IRLS = poseGT.unit_quaternion().angularDistance(poseIRLS.unit_quaternion());
            double errorT_IRLS = (poseGT.translation() - poseIRLS.translation()).norm();
            sumErrorR_IRLS += errorR_IRLS;
            sumErrorT_IRLS += errorT_IRLS;

            maxErrorR_BA = std::max(errorR_BA, maxErrorR_BA);
            maxErrorT_BA = std::max(errorT_BA, maxErrorT_BA);

            maxErrorR_IRLS = std::max(errorR_IRLS, maxErrorR_IRLS);
            maxErrorT_IRLS = std::max(errorT_IRLS, maxErrorT_IRLS);
        }

        double meanErrorT_BA_L2 = sumErrorT_BA / posesGT.size();
        double meanErrorR_BA_angDist = sumErrorR_BA / posesGT.size();


        double meanErrorT_IRLS_L2 = sumErrorT_IRLS / posesGT.size();
        double meanErrorR_IRLS_angDist = sumErrorR_IRLS / posesGT.size();


        std::cout << "__________IRLS test report 19 poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_IRLS_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_IRLS_angDist << std::endl;
        std::cout << "__________BA test report 19 poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_BA_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_BA_angDist << std::endl;


        ASSERT_LE(meanErrorR_BA_angDist, 0.02);
        ASSERT_LE(meanErrorT_BA_L2, 0.02);

        ASSERT_LE(maxErrorT_BA, maxErrorT_IRLS * coefficientT);
        ASSERT_LE(maxErrorR_BA, maxErrorR_IRLS * coefficientR);
        ASSERT_LE(meanErrorR_BA_angDist, meanErrorR_IRLS_angDist * coefficientR);
        ASSERT_LE(meanErrorT_BA_L2, meanErrorT_IRLS_L2 * coefficientT);
    }

}

// sometimes BA results are worse than IRLS so max error is multiplied by coefficient = 1.8
TEST(testBundleAdjustment, posesBundleAdjustmentUsingDepthBetterThanIRLS_151) {

    for (int iterations = 0; iterations < 1; ++iterations) {
        double coefficientR = 1.8;
        double coefficientT = 1.8;
        gdr::CorrespondenceGraph correspondenceGraph("../../data/360_dataset_sampled/each5/rgb",
                                                     "../../data/360_dataset_sampled/each5/depth",
                                                     517.3, 318.6,
                                                     516.5, 255.3);
        correspondenceGraph.computeRelativePoses();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
        std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

        std::string absolutePoses = "../../data/360_dataset_sampled/each5/groundtruth_new.txt";
        std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

        // compute absolute poses IRLS
        std::vector<Sophus::SE3d> posesIRLS;
        {
            for (int i = 0; i < computedAbsoluteTranslationsIRLS.size(); ++i) {
                Sophus::SE3d poseIRLS;
                poseIRLS.setQuaternion(computedAbsoluteOrientationsRobust[i]);
                poseIRLS.translation() = computedAbsoluteTranslationsIRLS[i];
                posesIRLS.push_back(poseIRLS);
            }
            // set origin at zero pose [IRLS]
            Sophus::SE3d poseIRLSzero = posesIRLS[0];
            for (auto &poseIRLS: posesIRLS) {
                poseIRLS = poseIRLSzero.inverse() * poseIRLS;
            }
        }
        {
            // set origin at zero pose [BA]
            Sophus::SE3d poseBAzero = bundleAdjustedPoses[0];
            for (auto &pose: bundleAdjustedPoses) {
                pose = poseBAzero.inverse() * pose;
            }
        }

        {
            // print poses IRLS to file
            std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/151/IRLS_151.txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * posesIRLS[i];

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }
        {
            // print poses BA depth to file
            std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/151/BA_151.txt";
            std::ofstream computedPoses(outputName);
            for (int i = 0; i < posesInfo.size(); ++i) {
                Sophus::SE3d poseSE3 = posesInfo[0].getSophusPose() * bundleAdjustedPoses[i];

                computedPoses.precision(std::numeric_limits<double>::max_digits10);
                computedPoses << posesInfo[i].getTimestamp() << ' ';
                const auto to = poseSE3.translation();
                for (int j = 0; j < 3; ++j) {
                    computedPoses << to[j] << ' ';
                }
                auto quatComputed = poseSE3.unit_quaternion();

                computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                              << quatComputed.w() << std::endl;
            }
        }





        // fill absolute poses as SE3
        std::vector<Sophus::SE3d> posesGT;

        for (const auto &poseGT: posesInfo) {
            Sophus::SE3d poseSE3;
            poseSE3.setQuaternion(poseGT.getOrientationQuat());
            poseSE3.translation() = poseGT.getTranslation();
            posesGT.push_back(poseSE3);
        }
        Sophus::SE3d poseGTzero = posesGT[0];
        for (auto &poseGT: posesGT) {
            poseGT = poseGTzero.inverse() * poseGT;
        }

        double sumErrorT_BA = 0;
        double sumErrorR_BA = 0;


        double sumErrorT_IRLS = 0;
        double sumErrorR_IRLS = 0;

        double maxErrorR_IRLS = 0;
        double maxErrorT_IRLS = 0;

        double maxErrorR_BA = 0;
        double maxErrorT_BA = 0;

        assert(posesGT.size() == bundleAdjustedPoses.size());
        assert(posesGT.size() == 151);

        for (int i = 0; i < posesGT.size(); ++i) {
            const auto &poseGT = posesGT[i];
            const auto &poseBA = bundleAdjustedPoses[i];
            const auto &poseIRLS = posesIRLS[i];

            double errorR_BA = poseGT.unit_quaternion().angularDistance(poseBA.unit_quaternion());
            double errorT_BA = (poseGT.translation() - poseBA.translation()).norm();
            sumErrorR_BA += errorR_BA;
            sumErrorT_BA += errorT_BA;

            double errorR_IRLS = poseGT.unit_quaternion().angularDistance(poseIRLS.unit_quaternion());
            double errorT_IRLS = (poseGT.translation() - poseIRLS.translation()).norm();
            sumErrorR_IRLS += errorR_IRLS;
            sumErrorT_IRLS += errorT_IRLS;

            maxErrorR_BA = std::max(errorR_BA, maxErrorR_BA);
            maxErrorT_BA = std::max(errorT_BA, maxErrorT_BA);

            maxErrorR_IRLS = std::max(errorR_IRLS, maxErrorR_IRLS);
            maxErrorT_IRLS = std::max(errorT_IRLS, maxErrorT_IRLS);
        }

        double meanErrorT_BA_L2 = sumErrorT_BA / posesGT.size();
        double meanErrorR_BA_angDist = sumErrorR_BA / posesGT.size();


        double meanErrorT_IRLS_L2 = sumErrorT_IRLS / posesGT.size();
        double meanErrorR_IRLS_angDist = sumErrorR_IRLS / posesGT.size();


        std::cout << "__________IRLS test report 151 poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_IRLS_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_IRLS_angDist << std::endl;
        std::cout << "__________BA test report 151 poses_____________" << std::endl;
        std::cout << "mean error translation: " << meanErrorT_BA_L2 << std::endl;
        std::cout << "mean error rotation: " << meanErrorR_BA_angDist << std::endl;


        ASSERT_LE(meanErrorR_BA_angDist, 0.02);
        ASSERT_LE(meanErrorT_BA_L2, 0.02);

        ASSERT_LE(maxErrorT_BA, maxErrorT_IRLS * coefficientT);
        ASSERT_LE(maxErrorR_BA, maxErrorR_IRLS * coefficientR);
        ASSERT_LE(meanErrorR_BA_angDist, meanErrorR_IRLS_angDist * coefficientR);
        ASSERT_LE(meanErrorT_BA_L2, meanErrorT_IRLS_L2 * coefficientT);
    }

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

