//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_DATASETCAMERADESCRIBER_H
#define GDR_DATASETCAMERADESCRIBER_H

#include <map>
#include <string>

#include "cameraModel/CameraRGBD.h"

namespace gdr {
    class DatasetCameraDescriber {

        std::map<std::string, CameraRGBD> cameraModelByImageRGBFilename;
        std::map<std::string, CameraRGBD> cameraModelByImageDFilename;

        CameraRGBD defaultCamera;

    public:

        DatasetCameraDescriber(const CameraRGBD &defaultCameraRgbd);

        void addCameraRgb(const std::string &filenameRgb, const CameraRGBD &camera);

        void addCameraDepth(const std::string &filenameDepth, const CameraRGBD &camera);

        const CameraRGBD &findCameraRgb(const std::string &filenameRgb, bool &found) const;

        const CameraRGBD &findCameraDepth(const std::string &filenameDepth, bool &found) const;

        const CameraRGBD &getDefaultCamera() const;

    };
}


#endif
