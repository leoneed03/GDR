//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_THREADPOOL_H
#define GDR_THREADPOOL_H

#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>

namespace gdr {

    class ThreadPoolTBB {

        tbb::task_scheduler_init sheduler;
    public:
        explicit ThreadPoolTBB(int numOfThreads);

        ThreadPoolTBB() = default;
    };
}

#endif
