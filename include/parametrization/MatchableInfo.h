//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_MATCHABLEINFO_H
#define GDR_MATCHABLEINFO_H

#include <string>
#include <vector>

#include "keyPoints/KeyPoint2DAndDepth.h"
#include "CameraRGBD.h"


namespace gdr {

    class MatchableInfo {

        int imagePixelHeight = 480;
        int imagePixelWidght = 640;

        std::string pathImageRGB;
        CameraRGBD cameraRGB;
        std::string pathImageD;
        std::vector<KeyPoint2DAndDepth> keyPoints2D;

    public:

        MatchableInfo(const std::string &pathRGB,
                      const std::string &pathD,
                      const std::vector<KeyPoint2DAndDepth> &keyPoints2D,
                      const CameraRGBD &cameraRGB);

        MatchableInfo() = default;

        const std::string &getPathImageRGB() const;

        const std::string &getPathImageD() const;

        const std::vector<KeyPoint2DAndDepth> getKeyPoints2D() const;

        const CameraRGBD& getCameraRGB() const;

        int getImagePixelHeight() const;

        int getImagePixelWidth() const;

        void setImagePixelHeightWidth(int pixelsHeight, int pixelsWidth);

    };
}

#endif
