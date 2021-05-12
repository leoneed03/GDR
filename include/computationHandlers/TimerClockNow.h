//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_TIMERCLOCKNOW_H
#define GDR_TIMERCLOCKNOW_H

#include <chrono>

namespace gdr {
    std::chrono::high_resolution_clock::time_point timerGetClockTimeNow();
}


#endif
