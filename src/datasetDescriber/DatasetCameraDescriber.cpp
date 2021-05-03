//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "datasetDescriber/DatasetCameraDescriber.h"

namespace gdr {

    DatasetCameraDescriber::DatasetCameraDescriber(const DatasetStructure &datasetStructureToSet,
                                                   const CameraRGBD &defaultCameraRgbd) :
            defaultCamera(defaultCameraRgbd),
            datasetStructure(datasetStructureToSet) {}

    void DatasetCameraDescriber::addCameraRgb(const std::string &filenameRgb,
                                              const CameraRGBD &camera) {

        cameraModelByImageRGBFilename[filenameRgb] = camera;
    }

    void DatasetCameraDescriber::addCameraDepth(const std::string &filenameDepth, const CameraRGBD &camera) {

        cameraModelByImageDFilename[filenameDepth] = camera;
    }

    const CameraRGBD &DatasetCameraDescriber::findCameraRgb(const std::string &filenameRgb, bool &found) const {

        const auto &camerasMap = cameraModelByImageRGBFilename;
        auto foundCamera = camerasMap.find(filenameRgb);

        if (foundCamera == camerasMap.end()) {
            return defaultCamera;
        }

        return foundCamera->second;
    }

    const CameraRGBD &DatasetCameraDescriber::findCameraDepth(const std::string &filenameDepth, bool &found) const {

        const auto &camerasMap = cameraModelByImageDFilename;
        auto foundCamera = camerasMap.find(filenameDepth);

        if (foundCamera == camerasMap.end()) {
            return defaultCamera;
        }

        return foundCamera->second;
    }

    const CameraRGBD &DatasetCameraDescriber::getDefaultCamera() const {
        return defaultCamera;
    }

    const DatasetStructure &DatasetCameraDescriber::getDatasetStructure() const {
        return datasetStructure;
    }

}
