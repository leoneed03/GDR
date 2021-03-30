//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "poseGraph/VertexCG.h"

namespace gdr {

    const std::vector<KeyPoint2DAndDepth> &VertexCG::getKeyPoints() const {
        return keypoints;
    }

    const std::vector<float> &VertexCG::getDescriptors() const {
        return descriptors;
    }

    const std::vector<double> &VertexCG::getDepths() const {
        return depths;
    }

    std::string VertexCG::getPathRGBImage() const {
        return pathToRGBimage;
    }

    std::string VertexCG::getPathDImage() const {
        return pathToDimage;
    }

    const CameraRGBD &VertexCG::getCamera() const {
        return cameraRgbd;
    }

    int VertexCG::getIndex() const {
        return index;
    }


    void VertexCG::setRotation(const Sophus::SO3d &rotation) {
        absolutePose.setRotation(rotation);
    }

    void VertexCG::setRotation(const SO3 &rotation) {
        absolutePose.setRotation(rotation);
    }

    void VertexCG::setRotation(const Eigen::Matrix3d &rotation) {
        Eigen::Quaterniond quatRotation(rotation);
        absolutePose.setRotation(Sophus::SO3d(quatRotation.normalized()));
    }

    void VertexCG::setRotation(const Eigen::Quaterniond &rotationQuatd) {
        absolutePose.setRotation(Sophus::SO3d(rotationQuatd.normalized()));
    }

    void VertexCG::setTranslation(const Eigen::Vector3d &translationToSet) {
        absolutePose.setTranslation(translationToSet);
        double error = (absolutePose.getTranslation() - translationToSet).norm();
        int coeffForEps = 100;
        assert(error < coeffForEps * std::numeric_limits<double>::epsilon());
    }

    void VertexCG::setRotationTranslation(const Eigen::Matrix4d &eigenRt) {
        absolutePose = SE3(Sophus::SE3d::fitToSE3(eigenRt));
    }

    void VertexCG::setRotationTranslation(const Sophus::SE3d &sophusRt) {
        absolutePose = SE3(sophusRt);
    }

    Eigen::Matrix4d VertexCG::getEigenMatrixAbsolutePose4d() const {
        return absolutePose.getSE3().matrix();
    }

    const SE3 &VertexCG::getAbsolutePoseSE3() const {
        return absolutePose;
    }


    const std::vector<KeyPoint2DAndDepth> &VertexCG::getKeyPoints2D() const {
        return keypoints;
    }

    Eigen::Quaterniond VertexCG::getRotationQuat() const {
        return absolutePose.getSE3().unit_quaternion();
    }


    Sophus::SE3d VertexCG::getAbsolutePoseSophus() const {
        return absolutePose.getSE3();
    }

    VertexCG::VertexCG(int newIndex,
                       const CameraRGBD &newCameraRgbd,
                       const keyPointsDepthDescriptor &keyPointsDepthDescriptor,
                       const std::string &newPathRGB,
                       const std::string &newPathD) : index(newIndex),
                                                      initialIndex(newIndex),
                                                      cameraRgbd(newCameraRgbd),
                                                      keypoints(keyPointsDepthDescriptor.getKeyPointsKnownDepth()),
                                                      descriptors(keyPointsDepthDescriptor.getDescriptorsKnownDepth()),
                                                      depths(keyPointsDepthDescriptor.getDepths()),
                                                      pathToRGBimage(newPathRGB),
                                                      pathToDimage(newPathD) {}

    void VertexCG::setIndex(int newIndex) {
        index = newIndex;
    }

    void VertexCG::setAbsolutePoseSE3(const SE3 &absolutePoseToSet) {
        absolutePose = absolutePoseToSet;
    }

    void VertexCG::setCamera(const CameraRGBD &camera) {
        cameraRgbd = camera;
    }

    int VertexCG::getInitialIndex() const {
        return initialIndex;
    }

    int VertexCG::getKeyPointsNumber() const {
        return keypoints.size();
    }

    const KeyPoint2DAndDepth &VertexCG::getKeyPoint(int keyPointIndex) const {
        assert(keyPointIndexIsValid(keyPointIndex));

        return keypoints[keyPointIndex];
    }

    bool VertexCG::keyPointIndexIsValid(int keyPointIndex) const {
        assert(keyPointIndex >= 0);
        assert(keyPointIndex < keypoints.size());
        int keyPointsNum = keypoints.size();
        assert(keyPointsNum == depths.size());
        assert(keyPointsNum * 128 == descriptors.size());

        return true;
    }

    double VertexCG::getKeyPointDepth(int keyPointIndex) const {
        assert(keyPointIndexIsValid(keyPointIndex));

        return depths[keyPointIndex];
    }
}
