//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <gtest/gtest.h>
#include <vector>
#include <random>

#include "readerTUM/ReaderTum.h"
#include "absolutePoseEstimation/translationAveraging/TranslationAverager.h"


TEST(testTranslationAveraging, translationRecovery19PosesFromFileUsingSE3) {
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(absolutePoses);

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
//        newAbsolutePose.block<3, 3>(0, 0) = absolutePoseEntry.unit_quaternion().toRotationMatrix();
        absolutePosesOnlyRotations.emplace_back(newAbsolutePose);
    }
    std::vector<gdr::translationMeasurement> relativeTs;

    for (int i = 0; i < posesInfo.size() - 1; ++i) {
        gdr::translationMeasurement relT(
                (absolutePosesFromGroundTruth[i].inverse() * absolutePosesFromGroundTruth[i + 1]).translation(), i,
                i + 1);
        relativeTs.push_back(relT);
    }
    gdr::Vectors3d absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                        absolutePosesOnlyRotations);
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations.toVectorOfVectors();

    for (int i = 0; i < absolutePosesFromGroundTruth.size(); ++i) {
        double errorT = (absolutePosesFromGroundTruth[i].translation() -
                         (absoluteTranslationsFirstZero[i] - absoluteTranslationsFirstZero[0])).norm();
        std::cout << "pose " << i << " error: " << errorT << std::endl;
        std::cout << "computed " << absoluteTranslationsFirstZero[i] << std::endl;
        ASSERT_LE(errorT, 1e-10);
    }

}


TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex2NoOutliers) {
    int dim = 3;
    int numPoses = 19;
    std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
            absolutePosesFile);
    std::vector<Sophus::SE3d> posesGT;

    for (const auto &poseGT: absolutePosesInfo) {
        Sophus::SE3d poseSE3;
        poseSE3.setQuaternion(poseGT.getOrientationQuat());
        poseSE3.translation() = poseGT.getTranslation();
        posesGT.push_back(poseSE3);
    }
    Sophus::SE3d poseGTzero = posesGT[0];
    for (auto &poseGT: posesGT) {
        poseGT = poseGTzero.inverse() * poseGT;
    }
    assert(posesGT.size() == numPoses);

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: posesGT) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry.unit_quaternion()
                        .toRotationMatrix();
        absolutePoses.emplace_back(newAbsolutePose);

    }

    double maxTranslationCoord = 1;
    std::uniform_real_distribution<> distrib(0, maxTranslationCoord);
    for (int indexFrom = 0; indexFrom < posesGT.size() - 1; ++indexFrom) {
        for (int indexTo = indexFrom + 1; indexTo < posesGT.size(); ++indexTo) {
            if (indexFrom + 1 == indexTo || indexFrom + 2 == indexTo) {
                gdr::translationMeasurement relPose((posesGT[indexFrom].inverse() * posesGT[indexTo]).translation(),
                                                    indexFrom, indexTo);
                relativeTs.push_back(relPose);
            }
        }
    }

    bool successIRLS = true;
    std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                      absolutePoses).toVectorOfVectors();
//        absoluteTranslations = gdr::translationAverager::recoverTranslationsIRLS(
//                relativeTs,
//                absolutePoses,
//                absoluteTranslations,
//                successIRLS).toVectorOfVectors();

    assert(absoluteTranslations.size() == numPoses);
    assert(absoluteTranslations.size() == absolutePosesInfo.size());
    std::vector<Eigen::Vector3d> absoluteTranslationsFirstZero = absoluteTranslations;

    for (auto &movedTranslation: absoluteTranslationsFirstZero) {
        movedTranslation -= absoluteTranslations[0];
    }

    double sumError = 0;
    for (int j = 0; j < absoluteTranslationsFirstZero.size(); ++j) {
        double error = (absoluteTranslationsFirstZero[j] - posesGT[j].translation()).norm();
        std::cout << j << " cur " << error << std::endl;
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    std::cout << "error is " << averageError << std::endl;
    ASSERT_LE(averageError, 1e-3);

}

TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex4NoOutliersIRLS) {

    int dim = 3;
    int numPoses = 19;
    std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

    std::vector<gdr::poseInfo> absolutePosesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
            absolutePosesFile);
    std::vector<Sophus::SE3d> posesGT;

    for (const auto &poseGT: absolutePosesInfo) {
        Sophus::SE3d poseSE3;
        poseSE3.setQuaternion(poseGT.getOrientationQuat());
        poseSE3.translation() = poseGT.getTranslation();
        posesGT.push_back(poseSE3);
    }
    Sophus::SE3d poseGTzero = posesGT[0];
    for (auto &poseGT: posesGT) {
        poseGT = poseGTzero.inverse() * poseGT;
    }
    assert(posesGT.size() == numPoses);

    std::vector<gdr::translationMeasurement> relativeTs;
    std::vector<Eigen::Matrix4d> absolutePoses;
    for (const auto &absolutePoseEntry: posesGT) {
        Eigen::Matrix4d newAbsolutePose;
        newAbsolutePose.setIdentity();
        newAbsolutePose.block<3, 3>(0, 0) =
                absolutePoseEntry.unit_quaternion()
                        .toRotationMatrix();
        absolutePoses.emplace_back(newAbsolutePose);

    }


    std::random_device randomDevice;
    std::mt19937 randomNumberGenerator(randomDevice());
    double maxTranslationCoord = 1;
    std::uniform_real_distribution<> distrib(0, maxTranslationCoord);
    for (int indexFrom = 0; indexFrom < posesGT.size() - 1; ++indexFrom) {
        for (int indexTo = indexFrom + 1; indexTo < posesGT.size(); ++indexTo) {
            if (indexFrom + 1 == indexTo || indexFrom + 2 == indexTo) {
                gdr::translationMeasurement relPose((posesGT[indexFrom].inverse() * posesGT[indexTo]).translation(),
                                                    indexFrom, indexTo);
                relativeTs.push_back(relPose);
            }
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
    }

    double sumError = 0;
    for (int j = 0; j < absoluteTranslationsFirstZero.size(); ++j) {
        double error = (absoluteTranslationsFirstZero[j] - posesGT[j].translation()).norm();
        std::cout << j << " cur " << error << std::endl;
        sumError += error;
    }

    double averageError = sumError / absoluteTranslationsFirstZero.size();

    std::cout << "error is " << averageError << std::endl;
    ASSERT_LE(averageError, 1e-10);

}


TEST(testTranslationAveraging, IRLS19PosesFromFileCorrespondencesPerVertex4SomeOutliers) {

    for (int iterations = 0; iterations < 20; ++iterations) {

        int numPoses = 19;
        std::string absolutePosesFile = "../../data/files/absolutePoses_" + std::to_string(numPoses) + ".txt";

        std::vector<gdr::poseInfo> absolutePosesInfo = gdr::ReaderTUM::getPoseInfoTimeTranslationOrientation(
                absolutePosesFile);
        std::vector<Sophus::SE3d> posesGT;

        for (const auto& poseGT: absolutePosesInfo) {
            Sophus::SE3d poseSE3;
            poseSE3.setQuaternion(poseGT.getOrientationQuat());
            poseSE3.translation() = poseGT.getTranslation();
            posesGT.push_back(poseSE3);
        }
        Sophus::SE3d poseGTzero = posesGT[0];
        for (auto& poseGT: posesGT) {
            poseGT = poseGTzero.inverse() * poseGT;
        }
        assert(posesGT.size() == numPoses);

        std::vector<gdr::translationMeasurement> relativeTs;
        std::vector<Eigen::Matrix4d> absolutePoses;
        for (const auto &absolutePoseEntry: posesGT) {
            Eigen::Matrix4d newAbsolutePose;
            newAbsolutePose.setIdentity();
            newAbsolutePose.block<3, 3>(0, 0) =
                    absolutePoseEntry.unit_quaternion()
                            .toRotationMatrix();
            absolutePoses.emplace_back(newAbsolutePose);

        }


        std::random_device randomDevice;
        std::mt19937 randomNumberGenerator(randomDevice());
        double maxTranslationCoord = 0.2;
        std::uniform_real_distribution<> distrib(-maxTranslationCoord, maxTranslationCoord);

        for (int indexFrom = 0; indexFrom < posesGT.size() - 1; ++indexFrom) {
            for (int indexTo = indexFrom + 1; indexTo < posesGT.size(); ++indexTo) {
                if (indexFrom + 1 == indexTo || indexFrom + 2 == indexTo || indexFrom + 3 == indexTo) {
                    gdr::translationMeasurement relPose((posesGT[indexFrom].inverse() * posesGT[indexTo]).translation(), indexFrom, indexTo);
                    relativeTs.push_back(relPose);
                }
                if (indexFrom + 4 == indexTo) {
                    Eigen::Vector3d outlierT;
                    outlierT[0] = distrib(randomNumberGenerator);
                    outlierT[1] = distrib(randomNumberGenerator);
                    outlierT[2] = distrib(randomNumberGenerator);
                    gdr::translationMeasurement relPose(outlierT, indexFrom, indexTo);
                    relativeTs.push_back(relPose);
                }
            }
        }

        bool successIRLS = true;
        std::vector<Eigen::Vector3d> absoluteTranslations = gdr::translationAverager::recoverTranslations(relativeTs,
                                                                                                          absolutePoses).toVectorOfVectors();
        std::vector<Eigen::Vector3d> translationsPCG = absoluteTranslations;
        absoluteTranslations = gdr::translationAverager::recoverTranslationsIRLS(
                relativeTs,
                absolutePoses,
                absoluteTranslations,
                successIRLS).toVectorOfVectors();

        assert(absoluteTranslations.size() == numPoses);
        assert(absoluteTranslations.size() == absolutePosesInfo.size());
        Eigen::Vector3d translationPCGzero = translationsPCG[0];
        Eigen::Vector3d translationZero = absoluteTranslations[0];

        for (auto &movedTranslation: absoluteTranslations) {
            movedTranslation -= translationZero;
        }
        for (auto &movedTranslation: translationsPCG) {
            movedTranslation -= translationPCGzero;
        }

        double sumError = 0;
        double sumErrorPCG = 0;
        for (int j = 0; j < absoluteTranslations.size(); ++j) {
            double error = (absoluteTranslations[j] - posesGT[j].translation()).norm();
            double errorPCG = (translationsPCG[j] - posesGT[j].translation()).norm();
            std::cout << j << " cur " << error << std::endl;
            sumError += error;
            sumErrorPCG += errorPCG;
        }

        double averageError = sumError / absoluteTranslations.size();
        double averageErrorPCG = sumErrorPCG / absoluteTranslations.size();

        std::cout << "error is " << averageError << std::endl;
        ASSERT_LE(averageError, 1e-1);
        ASSERT_LE(averageError, averageErrorPCG);
    }
}

int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}