//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TRAJECTORYFILESCOMPARER_H
#define GDR_TRAJECTORYFILESCOMPARER_H

#include <string>

namespace gdr {
    class TrajectoryFilesComparer {
    public:
        TrajectoryFilesComparer(const std::string &poseGroundtruthFile,
                                const std::string &poseTrajectoryEstimated);
    };
}


#endif
