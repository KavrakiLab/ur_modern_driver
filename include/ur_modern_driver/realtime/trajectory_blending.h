/*
 * trajectory_blending.h
 * Header file for a process that converts sns_msg_path_dense to
 * sns_msg_motor_state through parabolic blending.
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

#ifndef UR_MODERN_DRIVER_REALTIME_TRAJECTORY_BLENDING_H
#define UR_MODERN_DRIVER_REALTIME_TRAJECTORY_BLENDING_H

#include <amino.h>
#include <amino/ct/state.h>
#include <amino/ct/traj.h>

#include <sns.h>
#include <sns/event.h>
#include <sns/motor.h>
#include <ach/experimental.h>

/**
 * The event handler for trajectory blending: accepts waypoint paths
 * (sns_msg_path_dense) and sends the path off to the driver as a series of
 * sns_msg_motor_refs.
 */
enum ach_status handle_waypoint_path(void *cx_, void *msg_, size_t msg_size);



#endif // UR_MODERN_DRIVER_REALTIME_TRAJECTORY_BLENDING_H
