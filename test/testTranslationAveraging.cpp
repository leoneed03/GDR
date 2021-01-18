//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>

#include "groundTruthTransformer.h"
#include "translationAveraging.h"

TEST(testTranslationAveraging, translationRecovery2Poses) {

    gdr::translationMeasurement relativeT(Eigen::Vector3d(1, -1, 2), 0, 1);
    std::vector<gdr::translationMeasurement> relativeTs = {relativeT};
    std::vector<Eigen::Matrix4d> absolutePoses(2);
    absolutePoses[0].setIdentity();
    absolutePoses[1].setIdentity();
    gdr::translationAverager::recoverTranslations(relativeTs, absolutePoses);
    ASSERT_TRUE(true);
}

TEST(testTranslationAveraging, translationRecovery3Poses) {

    gdr::translationMeasurement relativeT0_1(Eigen::Vector3d(1, -1, 2), 0, 1);
    gdr::translationMeasurement relativeT0_2(Eigen::Vector3d(0, 10, -11), 0, 2);
    std::vector<gdr::translationMeasurement> relativeTs;
    relativeTs.push_back(relativeT0_1);
    relativeTs.push_back(relativeT0_2);
    std::vector<Eigen::Matrix4d> absolutePoses(3);
    for (auto &pose: absolutePoses) {
        pose.setIdentity();
    }
    gdr::translationAverager::recoverTranslations(relativeTs, absolutePoses);
    ASSERT_TRUE(true);
}

TEST(testTranslationAveraging, translationRecovery19PosesFromFile) {

    std::string absolutePosesFile = "../../data/files/absolutePoses_19.txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);
    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto& absolutePoseEntry: absolutePosesInfo) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry
                        .getOrientationQuat()
                        .normalized()
                        .toRotationMatrix();
        absolutePoses.emplace_back(newAbsolutePose);

    }

    for (const auto &relativePosePair: relativePoses) {
        relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
    }
    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses);
    std::cout << "after PCG got " << absoluteTranslations.size() << " poses" << std::endl;
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto& movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
        movedTranslation += absolutePosesInfo[0].getTranslation();
    }

    double sumError = 0;
    for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
        double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
        std::cout << "Pose " << i << " " << error << std::endl;
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    ASSERT_TRUE(averageError < 1e-10);
}


TEST(testTranslationAveraging, translationRecovery4PosesFromFile) {

    std::string absolutePosesFile = "../../data/files/absolutePoses_4.txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);
//
//    std::cout << "Poses " << absolutePosesInfo.size() << std::endl;
//    exit(0);

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto& absolutePoseEntry: absolutePosesInfo) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry
                        .getOrientationQuat()
                        .normalized()
                        .toRotationMatrix();
        absolutePoses.emplace_back(newAbsolutePose);

    }

    for (const auto &relativePosePair: relativePoses) {
        relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
    }

    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses);
    std::cout << "after PCG got " << absoluteTranslations.size() << " poses" << std::endl;
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto& movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
        movedTranslation += absolutePosesInfo[0].getTranslation();
    }

    double sumError = 0;
    for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
        double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
        std::cout << "Pose " << i << " " << error << std::endl;
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    ASSERT_TRUE(averageError < 1e-10);
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}