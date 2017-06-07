/*
 * trajectory_blending..cpp
 * A process that converts sns_msg_path_dense into sns_msg_motor_states through
 * parabolic blending in amino.
 *
 * Copyright 2017 Bryce Willey
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

#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "1.0"
#endif

#include "ur_modern_driver/realtime/trajectory_blending.h"

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx, 1);

    const double opt_sim_frequency = 100;

    // The UR driver expects messages as 'struct cx', I think.

    return 0;
}

enum ach_status handle_waypoint_path(void *cx_, void *msg_, size_t msg_size) {}

