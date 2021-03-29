//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include "parametrization/MatchableInfo.h"

namespace gdr {

    MatchableInfo::MatchableInfo(const std::string &pathRGB,
                                 const std::string &pathD,
                                 const std::vector<KeyPoint2DAndDepth> &keyPoints2DToSet,
                                 const CameraRGBD & cameraRGB) :
                                 pathImageRGB(pathRGB),
                                 pathImageD(pathD),
                                 keyPoints2D(keyPoints2DToSet),
                                 cameraRGB(cameraRGB) {}

    const std::string &MatchableInfo::getPathImageRGB() const {
        return pathImageRGB;
    }

    const std::string &MatchableInfo::getPathImageD() const {
        return pathImageD;
    }

    const std::vector<KeyPoint2DAndDepth> MatchableInfo::getKeyPoints2D() const {
        return keyPoints2D;
    }

    const CameraRGBD& MatchableInfo::getCameraRGB() const {
        return cameraRGB;
    }

    int MatchableInfo::getImagePixelHeight() const {
        return imagePixelHeight;
    }

    int MatchableInfo::getImagePixelWidth() const {
        return imagePixelWidght;
    }

    void MatchableInfo::setImagePixelHeightWidth(int pixelsHeight, int pixelsWidth) {
        assert(pixelsWidth > 0 && pixelsHeight > 0);
        imagePixelHeight = pixelsHeight;
        imagePixelWidght = pixelsWidth;
    }
}