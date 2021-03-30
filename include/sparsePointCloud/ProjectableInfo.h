//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_PROJECTABLEINFO_H
#define GDR_PROJECTABLEINFO_H

#include "parametrization/SE3.h"
#include "parametrization/cameraRGBD.h"

namespace gdr {

    class ProjectableInfo {
        SE3 poseSE3;
        CameraRGBD camera;
        int index;
        std::string pathRGB;
        std::string pathD;

    public:
        ProjectableInfo(const SE3 &poseSE3,
                        const CameraRGBD &camera,
                        int index,
                        const std::string &pathRGB,
                        const std::string &pathD);

        int getIndex() const;

        const CameraRGBD &getCamera() const;

        const SE3 &getPoseSE3() const;

        const std::string &getPathRGB() const;

        const std::string &getPathD() const;

        void setPoseSE3(const SE3 &poseSE3);

    };
}

#endif
