//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "VertexCG.h"

namespace gdr {


    VertexCG::VertexCG(int newIndex,
                       const CameraRGBD &newCameraRgbd,
                       const std::vector<SiftGPU::SiftKeypoint> &newKeypoints,
                       const std::vector<float> &newDescriptors,
                       const std::vector<double> &newDepths,
                       const std::string &newPathRGB,
                       const std::string &newPathD) : index(newIndex),
                                                      cameraRgbd(newCameraRgbd),
                                                      keypoints(newKeypoints),
                                                      descriptors(newDescriptors),
                                                      depths(newDepths),
                                                      pathToRGBimage(newPathRGB),
                                                      pathToDimage(newPathD) {
        absoluteRotationTranslation.setIdentity();

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


    void VertexCG::setRotation(const Eigen::Matrix3d &rotation) {
        Eigen::Quaterniond quatRotation(rotation);
        quatRotation.normalize();
        absolutePose.setQuaternion(quatRotation);
        absoluteRotationTranslation.block<3, 3>(0, 0) = rotation;
    }
    void VertexCG::setRotation(const Eigen::Quaterniond &rotationQuatd) {

        absolutePose.setQuaternion(rotationQuatd.normalized());
        absoluteRotationTranslation.block<3, 3>(0, 0) = rotationQuatd.normalized().toRotationMatrix();
    }

    void VertexCG::setTranslation(const Eigen::Vector3d &newTranslation) {
        absolutePose.translation() = newTranslation;
        double error = (absolutePose.translation() - newTranslation).norm();
        int coeffForEps = 100;
        assert(error < coeffForEps * std::numeric_limits<double>::epsilon());
        absoluteRotationTranslation.block<3, 1>(0, 3) = newTranslation;
    }

    void VertexCG::setRotationTranslation(const Eigen::Matrix4d &eigenRt) {
        absolutePose = Sophus::SE3d::fitToSE3(eigenRt);
        absoluteRotationTranslation = eigenRt;
//        absoluteRotationTranslation = absolutePose.matrix();
    }

    void VertexCG::setRotationTranslation(const Sophus::SE3d &sophusRt) {
        absolutePose = sophusRt;
        absoluteRotationTranslation = sophusRt.matrix();
    }

    Eigen::Matrix4d VertexCG::getEigenMatrixAbsolutePose4d() const {
        return absolutePose.matrix();
    }

    Eigen::Quaterniond VertexCG::getRotationQuat() const {
        return absolutePose.unit_quaternion();
    }
}
