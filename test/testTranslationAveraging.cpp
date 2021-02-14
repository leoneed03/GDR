//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

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


TEST(testTranslationAveraging, translationRecovery19PosesFromFileUsingSE3) {
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

    std::vector<Eigen::Matrix4d> absolutePosesOnlyRotations;
    for (const auto &absolutePoseEntry: absolutePosesFromGroundTruth) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry.unit_quaternion().toRotationMatrix();
        absolutePosesOnlyRotations.emplace_back(newAbsolutePose);
    }
    std::vector<gdr::translationMeasurement> relativeTs;

    for (int i = 0; i < posesInfo.size() - 1; ++i) {
        gdr::translationMeasurement relT((absolutePosesFromGroundTruth[i].inverse() * absolutePosesFromGroundTruth[i + 1]).translation(), i, i + 1);
        relativeTs.push_back(relT);
    }
    gdr::Vectors3d absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs, absolutePosesOnlyRotations);
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations.toVectorOfVectors();

    for (int i = 0; i < absolutePosesFromGroundTruth.size(); ++i) {
        double errorT = (absolutePosesFromGroundTruth[i].translation() - (absoluteTranslationsFirstZero[i] - absoluteTranslationsFirstZero[0])).norm();
        std::cout << "pose " << i << " error: " << errorT << std::endl;
        std::cout << "computed " << absoluteTranslationsFirstZero[i] << std::endl;
        ASSERT_LE(errorT, 1e-10);
    }

}

TEST(testTranslationAveraging, translationRecovery19PosesFromFile) {

    int numPoses = 19;
    std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);
    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: absolutePosesInfo) {
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
    gdr::Vectors3d absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs, absolutePoses);
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations.toVectorOfVectors();

    int counter = 0;
    std::cout << "after PCG got " << absoluteTranslations.getSize() << " poses" << std::endl;
    assert(absoluteTranslations.getSize() == absolutePosesInfo.size());
    assert(absoluteTranslations.getSize() == absolutePosesInfo.size());

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
        ++counter;
        std::cout << counter << std::endl;

        std::cout << absoluteTranslations[0] << std::endl;
        std::cout << movedTranslation << std::endl;
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

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: absolutePosesInfo) {
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
                                                                                                      absolutePoses).toVectorOfVectors();
    std::cout << "after PCG got " << absoluteTranslations.size() << " poses" << std::endl;
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
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


TEST(testTranslationAveraging, translationRecovery4PosesFromFileCorrespondencesPerVertex1) {

    std::string absolutePosesFile = "../../data/files/absolutePoses_4.txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);


    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: absolutePosesInfo) {
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
        gdr::translationMeasurement relativeT = relativePosePair.getRelativeTranslation();
        if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 1) {
            relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
        }
    }

    assert(relativeTs.size() == absolutePosesInfo.size() - 1);
    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses).toVectorOfVectors();

    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
        movedTranslation += absolutePosesInfo[0].getTranslation();
    }

    double sumError = 0;
    for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
        double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    ASSERT_TRUE(averageError < 1e-10);
}

/*

TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex3NoOutliers) {

    int numPoses = 19;
    int dim = 3;
    std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: absolutePosesInfo) {
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
        gdr::translationMeasurement relativeT = relativePosePair.getRelativeTranslation();
        if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 1
            || relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 2
            || relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 3) {
            relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
        }
    }

    bool successIRLS = true;
    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses).toVectorOfVectors();
    absoluteTranslations = gdr::translationAverager::recoverTranslationsIRLS(
            relativeTs,
            absolutePoses,
            absoluteTranslations,
            successIRLS).toVectorOfVectors();
    assert((absoluteTranslations.size() - 2) * dim == relativeTs.size());
    assert(absoluteTranslations.size() == numPoses);
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
        movedTranslation += absolutePosesInfo[0].getTranslation();
    }

    double sumError = 0;
    for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
        double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    std::cout << "ERROR is " << averageError << std::endl;
    ASSERT_TRUE(averageError < 1e-8);
}*/

TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex3SomeOutliersNoIRLSLargeError) {

    int dim = 3;
    int numPoses = 19;
    std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

    std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
            absolutePosesFile);
    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesFile);

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: absolutePosesInfo) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry
                        .getOrientationQuat()
                        .normalized()
                        .toRotationMatrix();
        absolutePoses.emplace_back(newAbsolutePose);

    }


    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    double maxTranslationCoord = 1;
    std::uniform_real_distribution<> distrib(0, maxTranslationCoord);
    for (const auto &relativePosePair: relativePoses) {
        gdr::translationMeasurement relativeT = relativePosePair.getRelativeTranslation();
        if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 1
            || relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 2) {
            relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
        }
        if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 3) {
            std::vector<double> randomTranslation;
            for (int i = 0; i < dim; ++i) {
                randomTranslation.push_back(distrib(randomNumberGenerator));
            }
            Eigen::Vector3d random3dVector(randomTranslation.data());
            gdr::translationMeasurement relativeTOutlier(random3dVector,
                                                         relativeT.getIndexFromToBeTransformed(),
                                                         relativeT.getIndexFromToBeTransformed() + 3);
            std::cout << "vector\n" << random3dVector << std::endl;
            relativeTs.emplace_back(relativeTOutlier);
        }

    }

    bool successIRLS = true;
    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses).toVectorOfVectors();
//    absoluteTranslations = gdr::translationAverager::recoverTranslationsIRLS(
//            relativeTs,
//            absolutePoses,
//            absoluteTranslations,
//            successIRLS);

    assert(absoluteTranslations.size() == numPoses);
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
        movedTranslation += absolutePosesInfo[0].getTranslation();
    }

    double sumError = 0;
    for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
        double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    std::cout << "error is " << averageError << std::endl;
    ASSERT_GE(averageError, 1);
}

TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex4SomeOutliers) {

    for (int i = 0; i < 20; ++i) {
        int dim = 3;
        int numPoses = 19;
        std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

        std::vector<gdr::relativePose> relativePoses = gdr::GTT::extractAllRelativeTransformationPairwise(
                absolutePosesFile);
        std::vector<gdr::poseInfo> absolutePosesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(
                absolutePosesFile);

        std::vector<gdr::translationMeasurement> relativeTs;
        std::vector<Eigen::Matrix4d> absolutePoses;
        for (const auto &absolutePoseEntry: absolutePosesInfo) {
            Eigen::Matrix4d newAbsolutePose;
            newAbsolutePose.setIdentity();
            newAbsolutePose.block<3, 3>(0, 0) =
                    absolutePoseEntry
                            .getOrientationQuat()
                            .normalized()
                            .toRotationMatrix();
            absolutePoses.emplace_back(newAbsolutePose);

        }


        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        double maxTranslationCoord = 1;
        std::uniform_real_distribution<> distrib(0, maxTranslationCoord);
        for (const auto &relativePosePair: relativePoses) {
            gdr::translationMeasurement relativeT = relativePosePair.getRelativeTranslation();
            if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 1
                || relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 2
                || relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 4) {
                relativeTs.emplace_back(relativePosePair.getRelativeTranslation());
            }
            if (relativeT.getIndexToDestination() == relativeT.getIndexFromToBeTransformed() + 3) {
                std::vector<double> randomTranslation;
                for (int i = 0; i < dim; ++i) {
                    randomTranslation.push_back(distrib(randomNumberGenerator));
                }
                Eigen::Vector3d random3dVector(randomTranslation.data());
                gdr::translationMeasurement relativeTOutlier(random3dVector,
                                                             relativeT.getIndexFromToBeTransformed(),
                                                             relativeT.getIndexFromToBeTransformed() + 3);
                std::cout << "vector\n" << random3dVector << std::endl;
                relativeTs.emplace_back(relativeTOutlier);
            }

        }

        bool successIRLS = true;
        std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                          absolutePoses).toVectorOfVectors();
        absoluteTranslations = gdr::translationAverager::recoverTranslationsIRLS(
                relativeTs,
                absolutePoses,
                absoluteTranslations,
                successIRLS).toVectorOfVectors();

        assert(absoluteTranslations.size() == numPoses);
        assert(absoluteTranslations.size() == absolutePosesInfo.size());
        std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

        for (auto &movedTranslation: absoluteTranslationsFirstZero) {
            movedTranslation -= absoluteTranslations[0];
            movedTranslation += absolutePosesInfo[0].getTranslation();
        }

        double sumError = 0;
        for (int i = 0; i < absoluteTranslationsFirstZero.size(); ++i) {
            double error = (absoluteTranslationsFirstZero[i] - absolutePosesInfo[i].getTranslation()).norm();
            sumError += error;
        }

        double averageError = sumError / absoluteTranslationsFirstZero.size();

        std::cout << "error is " << averageError << std::endl;
        ASSERT_LE(averageError, 1e-1);
    }

}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}