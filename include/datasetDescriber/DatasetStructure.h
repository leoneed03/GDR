//
// Created by leoneed on 03.05.2021.
//

#ifndef GDR_DATASETSTRUCTURE_H
#define GDR_DATASETSTRUCTURE_H


namespace gdr {
    struct DatasetStructure {
        std::vector <std::string> pathsImagesRgb;
        std::vector <std::string> pathsImagesDepth;
        std::vector <std::pair<double, double>> timestampsRgbDepth;
        std::vector <std::pair<int, int>> pairedIndicesRgbAndDepth;
        std::string pathAssocFileFull = "";
    };
}



#endif
