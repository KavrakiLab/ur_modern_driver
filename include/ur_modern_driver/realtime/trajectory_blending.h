/*
 * trajectory_blending.h
 * Header file for a process that converts sns_msg_path_dense to
 * sns_msg_motor_state through parabolic blending.
 * Inspired by code by Zak Kingston.
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
 * The context needed for continually following a trajectory.
 */
struct traj_follow_cx_init {
    /** The channel to send motor references to (ideally the actual robot driver). */
    struct ach_channel ref_channel;

    /** How to send motor references to the robot driver. */
    enum sns_motor_mode mode;

    /** The proportional gain when following a trajectory. */
    double k_p;
};

struct traj_follow_cx {
    /**
     * The initial parameters that this follow context can be seeded from the
     * blend context.
     */
    const struct traj_follow_cx_init *init;

    /**
     * The blended path to follow.
     */
    struct aa_ct_seg_list *seg_list;

    /**
     * The time that the execution of this path was started.
     */
    double start_time;
};

/**
 * The context needed for handling waypoint paths.
 */
struct traj_blend_cx {
    /** The limits of the robot for which the path is being blended. */
    struct aa_ct_state *limits;

    /**
     * Used to begin the process of trajectory following after the waypoints
     * are blended.
     */
    struct traj_follow_cx_init *follow_cx;
};

/**
 * Turns sns dense paths into amino waypoint lists.
 * reg is the amino memory region from which the waypoint list will be allocated.
 *
 * NOTE: the t0 and period fields of the path argument are not used: the given
 * path is simply blended, and motor refs are sent with the current time and
 * motor frequency instead.
 */
struct aa_ct_pt_list *sns_to_amino_path(struct aa_mem_region *reg, struct sns_msg_path_dense *path);

/**
 * \brief The event handler for trajectory blending.
 *
 * cx_ is a traj_blend_cx, and msg_ is a waypoint path (sns_msg_path_dense).
 * Blends the waypoints and sends the first sns_msg_motor_ref to the driver.
 */
enum ach_status handle_blend_waypoint(void *cx_, void *msg_, size_t msg_size);

/**
 * \brief The event handler for following a blended trajectory.
 *
 * cx_ is a traj_follow_cx, and msg_ is a sns_motor_state.
 * Exerts KD-control on the robot to ensure that it follows the blended
 * trajectory.
 */
enum ach_status handle_follow_state(void *cx_, void *msg_, size_t msg_size);

#endif // UR_MODERN_DRIVER_REALTIME_TRAJECTORY_BLENDING_H
