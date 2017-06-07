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

#include <sns/msg.h>
#include "ur_modern_driver/realtime/trajectory_blending.h"

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx, 1);

    const double opt_sim_frequency = 100;

    // The UR driver expects messages as 'struct cx', I think.

    return 0;
}

enum ach_status handle_blend_waypoint(void *cx_, void *msg_, size_t msg_size)
{
    struct traj_blend_cx *cx = (struct traj_blend_cx *)cx;
    struct sns_msg_path_dense *msg = (struct sns_msg_path_dense *)msg_;

    if (sns_msg_motor_state_check_size(msg, msg_size)) {
        SNS_LOG(LOG_ERR, "Mismatched message size on channel\n");
    } else {
        /* Turn this message into descritized position/velocity commands. */
        struct aa_mem_region reg;
        aa_mem_region_init(&reg, 512);
        struct aa_ct_pt_list *aa_list = sns_to_amino_path(reg, msg);

        // Blend the path.
        struct aa_ct_seg_list *seg_list = aa_ct_tjq_pb_generate(&reg, pt_list, cx->limits);

        // Initialize the follow context.
        struct traj_blend_cx follow_context;

    }
}

enum ach_status handle_follow_state(void *cx_, void *msg_, size_t msg_size)
{

}

struct aa_ct_pt_list *sns_to_amino_path(struct aa_mem_region *reg, struct sns_msg_path_dense *path)
{
    struct aa_ct_pt_list *aa_list = aa_ct_pt_list_create(reg);

    int n_steps = path->n_steps;
    int n_q = path->n_dof;
    for (int i = 0; i < n_steps; i++)
    {
        struct aa_ct_state *state = aa_ct_state_alloc(reg, n_q, 0);
        AA_MEM_CPY(state->q, path->x + i * n_steps, n_q);
        aa_ct_pt_list_add(aa_list, state);
    }

    return aa_list;
}
