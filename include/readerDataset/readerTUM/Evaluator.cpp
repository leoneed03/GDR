//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "Evaluator.h"
#include "ReaderTum.h"
#include "ClosestMatchFinder.h"

#include <algorithm>

namespace gdr {

    Evaluator::Evaluator(const std::string &groundtruthFile) {

        std::vector<PoseFullInfo> posesReadFromFile = ReaderTUM::getPoseInfoTimeTranslationOrientation(groundtruthFile);
        initFromPosesVector(posesReadFromFile);
    }

    Evaluator::Evaluator(const std::vector<PoseFullInfo> &poses) {
        initFromPosesVector(poses);
    }

    void Evaluator::initFromPosesVector(const std::vector<PoseFullInfo> &poses) {
        posesGroundTruth = poses;
        std::sort(posesGroundTruth.begin(), posesGroundTruth.end(),
                  [](const PoseFullInfo &lhs, const PoseFullInfo &rhs) {
                      return lhs.getTimestamp() < rhs.getTimestamp();
                  });

        for (const auto &pose: posesGroundTruth) {
            poseGroundTruthByTime.insert(std::make_pair(pose.getTimestamp(), pose));
        }
    }


    ErrorRotationTranslation Evaluator::evaluateTrajectory(const std::vector<PoseFullInfo> &trajectory,
                                                           int indexFixed,
                                                           double maxTimeDiff) const {


        auto setOfPosesFromGroundTruth = poseGroundTruthByTime;

        if (setOfPosesFromGroundTruth.empty()) {
            std::cout << "Provided ground thruth file is empty" << std::endl;
            return ErrorRotationTranslation({"", "", 0, 0, 0, 0, 0, 0}, {"", "", 0, 0, 0, 0, 0, 0});
        }

        double sumErrorL2 = 0;
        double sumErrorRadian = 0;
        double sumErrorL2Squared = 0;
        double sumErrorRadianSquared = 0;

        std::vector<double> errorsL2;
        std::vector<double> errorsRadians;
        std::vector<double> errorsL2Squared;
        std::vector<double> errorsRadiansSquared;

        auto closestMatchToFixedZero = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(
                setOfPosesFromGroundTruth, trajectory[indexFixed].getTimestamp()
        );
        assert(closestMatchToFixedZero != setOfPosesFromGroundTruth.end());

        auto fixedPoseGroundTruth(closestMatchToFixedZero->second.getSophusPose());

        SE3 zeroPoseGt;
        SE3 zeroPoseTrajectory;

        int iteration = 0;

        std::vector<PoseFullInfo> posesMatchedGroundTruth;
        std::vector<PoseFullInfo> posesMatchedTrajectory;


        for (const auto &poseTrajectory: trajectory) {

            auto closestMatch = ClosestMatchFinder::findClosestKeyMatch<double, PoseFullInfo>(
                    setOfPosesFromGroundTruth, poseTrajectory.getTimestamp()
            );
            assert(closestMatch != setOfPosesFromGroundTruth.end());


            double foundTimestamp = closestMatch->second.getTimestamp();
            double poseTimestamp = poseTrajectory.getTimestamp();

            if (std::abs(foundTimestamp - poseTimestamp) > maxTimeDiff) {
                std::cout.precision(std::numeric_limits<double>::max_digits10);
                std::cout << "found timestamp " << foundTimestamp
                          << " while looking for " << poseTimestamp << std::endl;
                continue;
            }

            posesMatchedGroundTruth.emplace_back(closestMatch->second);
            posesMatchedTrajectory.emplace_back(poseTrajectory);
        }

        Eigen::Matrix3Xd pointsGroundTruth(3, posesMatchedTrajectory.size());
        Eigen::Matrix3Xd pointsTrajectory(3, posesMatchedGroundTruth.size());

        for (int point = 0; point < posesMatchedGroundTruth.size(); ++point) {
            pointsGroundTruth.col(point) = posesMatchedGroundTruth[point].getTranslation();
            pointsTrajectory.col(point) = posesMatchedTrajectory[point].getTranslation();
        }

        //align trajectories using umeyama + fit (cR+t) to SE3
        SE3 transformation(Eigen::umeyama(pointsGroundTruth, pointsTrajectory));

        for (auto &pose: posesMatchedGroundTruth) {
            SE3 alignedGroundtruthPose(transformation * pose.getSophusPose());

            pose = PoseFullInfo(pose.getTimestamp(), alignedGroundtruthPose);
        }

        for (int pose = 0; pose < posesMatchedGroundTruth.size(); ++pose) {

            ++iteration;

            SE3 alignedGroundTruthPose(posesMatchedGroundTruth[pose].getSophusPose());

            std::pair<double, double> errorRotationTranslation =
                    alignedGroundTruthPose.getRotationTranslationErrors(posesMatchedTrajectory[pose].getSophusPose());

            double errorL2 = errorRotationTranslation.second;
            double errorRadian = errorRotationTranslation.first;

            errorsL2.emplace_back(errorL2);
            errorsRadians.emplace_back(errorRadian);

            errorsL2Squared.emplace_back(errorL2 * errorL2);
            errorsRadiansSquared.emplace_back(errorRadian * errorRadian);

            sumErrorL2 += errorL2;
            sumErrorRadian += errorRadian;

            sumErrorL2Squared += errorL2 * errorL2;
            sumErrorRadianSquared += errorRadian * errorRadian;
        }


        sort(errorsL2);
        sort(errorsRadians);
        sort(errorsL2Squared);
        sort(errorsRadiansSquared);

        assert(errorsRadians.size() == errorsRadiansSquared.size());
        assert(errorsRadians.size() == errorsL2.size());
        assert(errorsL2.size() == errorsL2Squared.size());
        assert(!errorsL2.empty());


        int successfulMatchings = errorsL2.size();

        double meanL2 = sumErrorL2 / successfulMatchings;
        double meanRadian = sumErrorRadian / successfulMatchings;
        double meanL2Squared = sumErrorL2Squared / successfulMatchings;
        double meanRadianSquared = sumErrorRadianSquared / successfulMatchings;

        double medianL2 = median(errorsL2);
        double medianRadian = median(errorsRadians);

        double rmseL2 = std::sqrt(meanL2Squared);
        double rmseRadian = std::sqrt(meanRadianSquared);

        double stdL2 = std::sqrt(meanL2Squared - std::pow(meanL2, 2));
        double stdRadian = std::sqrt(meanRadianSquared - std::pow(meanRadian, 2));

        ErrorInformation errorInformationRadian("absolute_rotation_error", "",
                                                rmseRadian,
                                                meanRadian,
                                                medianRadian,
                                                stdRadian,
                                                errorsRadians[0],
                                                errorsRadians[successfulMatchings - 1]);

        ErrorInformation errorInformationL2("absolute_translational_error", "m",
                                            rmseL2,
                                            meanL2,
                                            medianL2,
                                            stdL2,
                                            errorsL2[0],
                                            errorsL2[successfulMatchings - 1]);

        ErrorRotationTranslation errorsRotTransl(errorInformationRadian,
                                                 errorInformationL2);

        errorsRotTransl.numberOfPosesEvaluated = successfulMatchings;
        errorsRotTransl.numberOfPosesGroundTruth = posesGroundTruth.size();
        errorsRotTransl.numberOfPosesTrajectory = trajectory.size();

        return errorsRotTransl;
    }

    void Evaluator::sort(std::vector<double> &values) {
        std::sort(values.begin(), values.end());
    }

    double Evaluator::median(const std::vector<double> &values) {
        assert(std::is_sorted(values.begin(), values.end()));

        assert(!values.empty());

        int sizeVec = values.size();

        if (sizeVec % 2 == 0) {
            return 0.5 * (values[sizeVec / 2] + values[sizeVec / 2 + 1]);
        } else {
            return values[sizeVec / 2];
        }
    }

}
