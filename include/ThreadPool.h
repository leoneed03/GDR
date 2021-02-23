//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_THREADPOOL_H
#define GDR_THREADPOOL_H

#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>

namespace gdr {

    struct ThreadPool {

        tbb::task_scheduler_init sheduler;
        tbb::task_group tasks;

        ThreadPool(int numOfThreads);

        tbb::task_group &getTaskGroup();
    };
}

#endif
