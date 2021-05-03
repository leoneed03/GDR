//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "datasetDescriber/DatasetDescriber.h"

namespace gdr {

    DatasetDescriber::DatasetDescriber(const DatasetStructure &datasetStructureToSet,
                                       const CameraRGBD &defaultCameraRgbd) :
            defaultCamera(defaultCameraRgbd),
            datasetStructure(datasetStructureToSet) {}

    void DatasetDescriber::addCameraRgb(const std::string &filenameRgb,
                                        const CameraRGBD &camera) {

        cameraModelByImageRGBFilename[filenameRgb] = camera;
    }

    void DatasetDescriber::addCameraDepth(const std::string &filenameDepth, const CameraRGBD &camera) {

        cameraModelByImageDFilename[filenameDepth] = camera;
    }

    const CameraRGBD &DatasetDescriber::findCameraRgb(const std::string &filenameRgb, bool &found) const {

        const auto &camerasMap = cameraModelByImageRGBFilename;
        auto foundCamera = camerasMap.find(filenameRgb);

        if (foundCamera == camerasMap.end()) {
            return defaultCamera;
        }

        return foundCamera->second;
    }

    const CameraRGBD &DatasetDescriber::findCameraDepth(const std::string &filenameDepth, bool &found) const {

        const auto &camerasMap = cameraModelByImageDFilename;
        auto foundCamera = camerasMap.find(filenameDepth);

        if (foundCamera == camerasMap.end()) {
            return defaultCamera;
        }

        return foundCamera->second;
    }

    const CameraRGBD &DatasetDescriber::getDefaultCamera() const {
        return defaultCamera;
    }

    const DatasetStructure &DatasetDescriber::getDatasetStructure() const {
        return datasetStructure;
    }

}
