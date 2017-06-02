/*
 * ur_realtime_driver.h
 *
 * Copyright 2017 Cannon Lewis
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_MODERN_DRIVER_UR_REALTIME_DRIVER_H
#define UR_MODERN_DRIVER_UR_REALTIME_DRIVER_H

#include <poll.h>
#include <sns.h>
#include <sns/event.h>
#include <ach/experimental.h>
#include <getopt.h>

#include <cblas.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/mem.h>

#include <amino/rx/scene_gl.h>
#include <amino/rx/scene_win.h>

#include "ur_driver.h"

struct cx;

struct in_cx {
    struct ach_channel channel;
    const char *name;
    struct cx *cx;
};

struct cx {
    struct in_cx *in;
    struct ach_channel state_out;
    size_t n_ref;

    struct aa_rx_sg *scenegraph;

    double *q_ref;
    double *dq_ref;
    int have_q_ref;
    int have_dq_ref;
    struct timespec t;

    double *q_act;
    double *dq_act;
    size_t n_q;
    uint64_t seq;

    struct aa_rx_win * win;

    struct sns_evhandler *handlers;
    struct timespec period;

    // Robot control provided by ur_modern_driver
    UrDriver *robot;

    // Messaging condition variables for ur_driver
    std::condition_variable rt_msg_cond;
    std::condition_variable msg_cond;
};

// Run io
void io(struct cx *cx);
// Pthreads start function for io
void* io_start(void *cx);

// Call periodically from io thread
enum ach_status io_periodic( void *cx );

void put_state( struct cx *cx );

// Perform a step
enum ach_status command( struct cx *cx );

// Handle a message
enum ach_status handle_msg( void *cx, void *msg, size_t msg_size );

#endif //UR_MODERN_DRIVER_UR_REALTIME_DRIVER_H
