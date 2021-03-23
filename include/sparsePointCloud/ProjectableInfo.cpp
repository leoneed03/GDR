//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "ProjectableInfo.h"

namespace gdr {

    ProjectableInfo::ProjectableInfo(const SE3 &poseSE3ToSet,
                                     const CameraRGBD &cameraToSet,
                                     int indexToSet,
                                     const std::string &pathRGBToSet,
                                     const std::string &pathDToSet) :
            poseSE3(poseSE3ToSet),
            camera(cameraToSet),
            index(indexToSet),
            pathRGB(pathRGBToSet),
            pathD(pathDToSet) {}

    int ProjectableInfo::getIndex() const {
        return index;
    }

    const CameraRGBD &ProjectableInfo::getCamera() const {
        return camera;
    }

    const SE3 &ProjectableInfo::getPoseSE3() const {
        return poseSE3;
    }

    const std::string &ProjectableInfo::getPathRGB() const {
        return pathRGB;
    }

    const std::string &ProjectableInfo::getPathD() const {
        return pathD;
    }

    void ProjectableInfo::setPoseSE3(const SE3 &poseSE3ToSet) {
        poseSE3 = poseSE3ToSet;
    }
}
