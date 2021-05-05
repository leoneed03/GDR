//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include "computationHandlers/TimerClockNow.h"

namespace gdr {

    std::chrono::high_resolution_clock::time_point timerGetClockTimeNow() {
        return std::chrono::high_resolution_clock::now();
    }
}