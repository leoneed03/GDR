//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"


TEST(testBundleAdjustment, justIRLS) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
//    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);


    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_iterRLS_.txt";
    std::ofstream computedPoses(outputName);
    for (int i = 0; i < posesInfo.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
//        poseSE3 = poseSE3.inverse();
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

TEST(testBundleAdjustment, allPosesAreOptimized) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustment();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfo[i].getTranslation());
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double errorRot = 0;
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto quatShouldBeId = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();
        double dError = quatShouldBeId.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << "error is " << dError << std::endl;
        errorRot += dError;
    }
    std::cout << "Mean Rot angle error " << errorRot / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;
    auto zeroT = absoluteTranslationsFromGroundTruth[0];

    for (auto& translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroT;
    }

    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < absoluteTranslationsFromGroundTruth.size(); ++i) {
        const auto& t = absoluteTranslationsFromGroundTruth[i];
        const auto& to = computedAbsoluteTranslationsIRLS[i];
        std::cout << i << ": \t" << t[0] << " \t" << t[1] << " \t" << t[2] << std::endl;
        std::cout << " : \t" << to[0] << " \t" << to[1] << " \t" << to[2] << std::endl;
    }

    std::cout << "______________________________________________________________" << std::endl;



    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

//    std::string outputName = "absolutePoses_19_BA.txt";

    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_BA_only_ransac.txt";
    std::ofstream computedPoses(outputName);

    assert(computedAbsoluteTranslationsIRLS.size() == absoluteTranslationsFromGroundTruth.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absoluteTranslationsFromGroundTruth[i] - computedAbsoluteTranslationsIRLS[i]).norm();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        std::cout << i << ":\t" << currentL2Error << std::endl;
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto to = bundleAdjustedPoses[i].translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = bundleAdjustedPoses[i].unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();


    correspondenceGraph.printConnectionsRelative(std::cout);
    std::cout << "IRLS for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    ASSERT_LE(meanError, 0.15);
}










TEST(testBundleAdjustment, BundleAdjustedPosesAreBetterThanAveraged) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustment();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfo[i].getTranslation());
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());


    double errorRotRobust = 0;
    for (int i = 0; i < computedAbsoluteOrientationsRobust.size(); ++i) {
        const auto& quat = computedAbsoluteOrientationsRobust[i];
        double dErrorRobust = quat.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error robust is: " << dErrorRobust << std::endl;
        errorRotRobust += dErrorRobust;
    }


    double errorRotBA = 0;
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto quatBA = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();
        double dError = quatBA.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error BA is: " << dError << std::endl;
        errorRotBA += dError;
    }


    auto zeroT = absoluteTranslationsFromGroundTruth[0];

    for (auto& translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroT;
    }

    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < absoluteTranslationsFromGroundTruth.size(); ++i) {
        const auto& t = absoluteTranslationsFromGroundTruth[i];
        const auto& to = computedAbsoluteTranslationsIRLS[i];
        std::cout << i << ": \t" << t[0] << " \t" << t[1] << " \t" << t[2] << std::endl;
        std::cout << " : \t" << to[0] << " \t" << to[1] << " \t" << to[2] << std::endl;
    }

    std::cout << "______________________________________________________________" << std::endl;



    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    std::string outputName = "absolutePoses_19_BA.txt";
//    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/b/absolutePoses_19_BA_inversedProjections.txt";
    std::ofstream computedPoses(outputName);

    assert(computedAbsoluteTranslationsIRLS.size() == absoluteTranslationsFromGroundTruth.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absoluteTranslationsFromGroundTruth[i] - computedAbsoluteTranslationsIRLS[i]).norm();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        std::cout << i << ":\t" << currentL2Error << std::endl;
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto to = bundleAdjustedPoses[i].translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = bundleAdjustedPoses[i].unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();


    correspondenceGraph.printConnectionsRelative(std::cout);
    std::cout << "IRLS for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    std::cout << "Mean Rot angle error BA " << errorRotBA / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;
    std::cout << "Mean Rot angle error robust " << errorRotRobust / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;

    ASSERT_LE(errorRotBA, errorRotRobust);
    ASSERT_LE(meanError, 0.15);
}



TEST(testBundleAdjustment, BundleAdjustedUsingDepthPosesAreBetterThanAveraged) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfo[i].getTranslation());
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());


    double errorRotRobust = 0;
    for (int i = 0; i < computedAbsoluteOrientationsRobust.size(); ++i) {
        const auto& quat = computedAbsoluteOrientationsRobust[i];
        double dErrorRobust = quat.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error robust is: " << dErrorRobust << std::endl;
        errorRotRobust += dErrorRobust;
    }


    double errorRotBA = 0;
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto quatBA = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();
        double dError = quatBA.angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error BA is: " << dError << std::endl;
        errorRotBA += dError;
    }


    auto zeroT = absoluteTranslationsFromGroundTruth[0];

    for (auto& translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroT;
    }

    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < absoluteTranslationsFromGroundTruth.size(); ++i) {
        const auto& t = absoluteTranslationsFromGroundTruth[i];
        const auto& to = computedAbsoluteTranslationsIRLS[i];
        std::cout << i << ": \t" << t[0] << " \t" << t[1] << " \t" << t[2] << std::endl;
        std::cout << " : \t" << to[0] << " \t" << to[1] << " \t" << to[2] << std::endl;
    }

    std::cout << "______________________________________________________________" << std::endl;



    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

    std::string outputName = "absolutePoses_19_BA.txt";
//    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_computed_poses_inversed.txt";
    std::ofstream computedPoses(outputName);

    assert(computedAbsoluteTranslationsIRLS.size() == absoluteTranslationsFromGroundTruth.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absoluteTranslationsFromGroundTruth[i] - computedAbsoluteTranslationsIRLS[i]).norm();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        std::cout << i << ":\t" << currentL2Error << std::endl;
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        Sophus::SE3d poseInversed = bundleAdjustedPoses[i].inverse();
        const auto to = poseInversed.translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = poseInversed.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();


    correspondenceGraph.printConnectionsRelative(std::cout);
    std::cout << "IRLS for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    std::cout << "Mean Rot angle error BA " << errorRotBA / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;
    std::cout << "Mean Rot angle error robust " << errorRotRobust / correspondenceGraph.verticesOfCorrespondence.size() << std::endl;

    ASSERT_LE(errorRotBA, errorRotRobust);
    ASSERT_LE(meanError, 0.15);
}




TEST(testBundleAdjustment, ComputeAbsoluteRotErrorAndTranslationError) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> absolutePosesFromGroundTruth;
    Sophus::SE3d poseZeroGT;
    poseZeroGT.setQuaternion(posesInfo[0].getOrientationQuat().normalized());
    poseZeroGT.translation() = posesInfo[0].getTranslation();

    for (int i = 0; i < posesInfo.size(); ++i) {
        Sophus::SE3d absolutePoseGT;
        absolutePoseGT.setQuaternion(posesInfo[i].getOrientationQuat().normalized());
        absolutePoseGT.translation() = posesInfo[i].getTranslation();
        absolutePosesFromGroundTruth.push_back(poseZeroGT.inverse() * absolutePoseGT);
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double errorRotBA = 0;
    std::vector<Sophus::SE3d> posesSE3Inversed;
    assert(absolutePosesFromGroundTruth.size() == posesInfo.size());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
//        poseSE3 = poseSE3.inverse();
        posesSE3Inversed.push_back(poseSE3);
        double dError = poseSE3.unit_quaternion().angularDistance(absolutePosesFromGroundTruth[i].unit_quaternion());
//        double dError = poseSE3.unit_quaternion().angularDistance(posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- absolute rotation error BA is: " << dError << std::endl;
        errorRotBA += dError;
    }


    double sumErrors = 0;
    double sumErrorsSquared = 0;

    assert(posesSE3Inversed.size() == posesInfo.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absolutePosesFromGroundTruth[i].translation() - posesSE3Inversed[i].translation()).norm();
        std::cout << "pose " << i << " current translation error: " << currentL2Error << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "BA for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    double meanRotError = errorRotBA / correspondenceGraph.verticesOfCorrespondence.size();
    std::cout << "Mean Rot angle error BA " << meanRotError << std::endl;

    ASSERT_LE(meanRotError, 0.03);
    ASSERT_LE(meanError, 0.03);
}

TEST(testBundleAdjustment, BAerrorsLessThanIRLS) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();

    std::vector<Sophus::SE3d> posesIRLS;

    auto poseZeroIRLS = correspondenceGraph.verticesOfCorrespondence[0].getEigenMatrixAbsolutePose4d();
    Sophus::SE3d poseIRLSzeroSE3 = Sophus::SE3d::fitToSE3(poseZeroIRLS);
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
        posesIRLS.push_back(poseIRLSzeroSE3.inverse() * poseSE3);
    }

    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> absolutePosesFromGroundTruth;
    Sophus::SE3d poseZeroGT;
    poseZeroGT.setQuaternion(posesInfo[0].getOrientationQuat().normalized());
    poseZeroGT.translation() = posesInfo[0].getTranslation();

    for (int i = 0; i < posesInfo.size(); ++i) {
        Sophus::SE3d absolutePoseGT;
        absolutePoseGT.setQuaternion(posesInfo[i].getOrientationQuat().normalized());
        absolutePoseGT.translation() = posesInfo[i].getTranslation();
        absolutePosesFromGroundTruth.push_back(poseZeroGT.inverse() * absolutePoseGT);
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double errorRotBA = 0;
    double errorRotIRLS = 0;
    double errorTranslationIRLS = 0;
    double sumErrors = 0;
    double sumErrorsSquared = 0;
    assert(absolutePosesFromGroundTruth.size() == posesInfo.size());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
        double dError = poseSE3.unit_quaternion().angularDistance(absolutePosesFromGroundTruth[i].unit_quaternion());
        double rotIRLS = absolutePosesFromGroundTruth[i].unit_quaternion().angularDistance(posesIRLS[i].unit_quaternion());
        double translationIRLS = (absolutePosesFromGroundTruth[i].translation() - posesIRLS[i].translation()).norm();

        double currentL2Error = (absolutePosesFromGroundTruth[i].translation() - poseSE3.translation()).norm();
        std::cout << "pose " << i << " current translation BA error: " << currentL2Error << std::endl;
        std::cout << "          "  << " current translation IRLS error: " << translationIRLS << std::endl;
        std::cout << "          " << " current rotation BA error: " << dError << std::endl;
        std::cout << "          " << " current rotation IRLS error: " << rotIRLS << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

        errorRotBA += dError;
        errorRotIRLS += rotIRLS;
        errorTranslationIRLS += translationIRLS;

    }

    double meanErrorIRLStranslation = errorTranslationIRLS / posesInfo.size();
    double meanErrorIRLSrotation = errorRotIRLS / posesInfo.size();
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();
    double meanRotError = errorRotBA / correspondenceGraph.verticesOfCorrespondence.size();

    std::cout << "______________________IRLS REPORT______________________" << std::endl;

    std::cout << "E(error translation absolute) = " << meanErrorIRLStranslation << std::endl;
    std::cout << "E(error rotation absolute) = " << meanErrorIRLSrotation << std::endl;
    std::cout << "______________________BA REPORT______________________" << std::endl;
    std::cout << "BA for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    std::cout << "Mean Rot angle error BA " << meanRotError << std::endl;

    ASSERT_LE(meanRotError, 0.03);
    ASSERT_LE(meanError, 0.03);
    ASSERT_LE(meanRotError, meanErrorIRLSrotation);
    ASSERT_LE(meanError, meanErrorIRLStranslation);
    ASSERT_LE(meanErrorIRLStranslation, 0.1);
    ASSERT_LE(meanErrorIRLSrotation, 0.15);
}


TEST(testBundleAdjustment, ComputeAbsoluteRotErrorAndTranslationErrorNoBA) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb", "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();

    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Sophus::SE3d> absolutePosesFromGroundTruth;
    Sophus::SE3d poseZeroGT;
    poseZeroGT.setQuaternion(posesInfo[0].getOrientationQuat().normalized());
    poseZeroGT.translation() = posesInfo[0].getTranslation();

    for (int i = 0; i < posesInfo.size(); ++i) {
        Sophus::SE3d absolutePoseGT;
        absolutePoseGT.setQuaternion(posesInfo[i].getOrientationQuat().normalized());
        absolutePoseGT.translation() = posesInfo[i].getTranslation();
        absolutePosesFromGroundTruth.push_back(poseZeroGT.inverse() * absolutePoseGT);
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());

    double errorRotBA = 0;
    std::vector<Sophus::SE3d> posesSE3Inversed;
    assert(absolutePosesFromGroundTruth.size() == posesInfo.size());


    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
        poseSE3 = poseSE3.inverse();
        posesSE3Inversed.push_back(poseSE3);
        double dError = poseSE3.unit_quaternion().angularDistance(absolutePosesFromGroundTruth[i].unit_quaternion());
        std::cout << i << " -- absolute rotation error BA is: " << dError << std::endl;
        errorRotBA += dError;
    }


    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/ba/absolutePoses_19_BA.txt";
    std::ofstream computedPoses(outputName);
    for (int i = 0; i < posesInfo.size(); ++i) {
        auto pose = correspondenceGraph.verticesOfCorrespondence[i].getEigenMatrixAbsolutePose4d();
        Sophus::SE3d poseSE3 = Sophus::SE3d::fitToSE3(pose);
//        poseSE3 = poseSE3.inverse();
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


    double sumErrors = 0;
    double sumErrorsSquared = 0;

    assert(posesSE3Inversed.size() == posesInfo.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absolutePosesFromGroundTruth[i].translation() - posesSE3Inversed[i].translation()).norm();
        std::cout << "pose " << i << " current translation error: " << currentL2Error << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();

    std::cout << "BA for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    double meanRotError = errorRotBA / correspondenceGraph.verticesOfCorrespondence.size();
    std::cout << "Mean Rot angle error BA " << meanRotError << std::endl;

    ASSERT_LE(meanRotError, 0.1);
    ASSERT_LE(meanError, 0.15);
}







int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

