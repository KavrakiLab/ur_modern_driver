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
 * See the License for the nowific language governing permissions and
 * limitations under the License.
 */

#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "1.0"
#endif

#include "ur_modern_driver/realtime/trajectory_blending.h"

int main(int argc, char **argv)
{
    struct traj_blend_cx blend_cx;
    struct traj_follow_cx follow_cx;
    AA_MEM_ZERO(&blend_cx, 1);
    AA_MEM_ZERO(&follow_cx, 1);

    const double opt_sim_frequency = 100;
    struct sns_motor_channel *last_mc = NULL;

    char *path_channel_name = NULL;
    /* Parse options. */
    {
        int c = 0;
        opterr = 0;
        while ( (c = getopt( argc, argv, "y:u:p:r:h?" SNS_OPTSTRING)) != -1) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-pblend",
                                      "Copyright (c) 2017, Rice University\n",
                                      "Bryce Willey")
                case 'y':
                    sns_motor_channel_push(optarg, &follow_cx.state_in);
                    last_mc = follow_cx.state_in;
                    break;
                case 'u':
                    sns_motor_channel_push(optarg, &follow_cx.ref_out);
                    last_mc = follow_cx.ref_out;
                    break;
                case 'w':
                    path_channel_name = optarg;
                    break;
                case 'p':
                    if (last_mc) {
                        last_mc->priority = atoi(optarg);
                    } else {
                        SNS_DIE("No channel specificd or priority argument.");
                    }
                    break;
                case '?':
                case 'h':
                    puts ("Usage: sns-pblend -u REF_CHANNEL -y STATE_CHANNEL -w PATH_CHANNEL\n"
                                  "Kinematically simulate a robot.\n"
                                  "\n"
                                  "Options:\n"
                                  "  -y <channel>,             state input channel\n"
                                  "  -u <channel>,             reference output channel\n"
                                  "  -w <channel>,             waypoint path input channel\n"
                                  "  -p <priority>,            channel priority\n"
                                  "  -V,                       Print program version\n"
                                  "  -?/-h,                    display this help and exit\n"
                                  "\n"
                                  "Examples:\n"
                                  "  sns-ksim -y state -u ref -w path\n"
                                  "\n"
                                  "Report bugs to <bsw2@rice.edu>"
                    );
                    exit(EXIT_SUCCESS);
                default:
                    SNS_DIE("Unknown option: '%c'\n", c);
                    break;
            }
        }
    }

    sns_init();

    SNS_REQUIRE(follow_cx.state_in, "Need state channel");
    SNS_REQUIRE(follow_cx.ref_out, "Need reference channel");
    SNS_REQUIRE(path_channel_name != NULL, "Need path channel");

    //aa_mem_region_init(&follow_cx.reg, 512);
    follow_cx.reg = aa_mem_region_local_get();

    struct aa_rx_sg *scenegraph = sns_scene_load();
    sns_motor_state_init(scenegraph, follow_cx.state_in, &follow_cx.state_set, 0, NULL);
    sns_motor_ref_init(scenegraph, follow_cx.ref_out, &follow_cx.ref_set, 0, NULL);

    /* Get the robot limits. */
    size_t config_count = aa_rx_sg_config_count(scenegraph);
    follow_cx.n_q = config_count;
    const char **names = (const char **)malloc(sizeof(char *) * config_count);
    aa_rx_sg_config_names(scenegraph, config_count, names);

    struct aa_ct_state *limits = aa_ct_state_alloc(follow_cx.reg, follow_cx.n_q, 0);
    for (size_t i = 0; i < config_count; i++) {
        double qmax, qmin, dqmax, dqmin, ddqmax, ddqmin;
        aa_rx_config_id id = aa_rx_sg_config_id(scenegraph, names[i]);
        aa_rx_sg_get_limit_pos(scenegraph, id, &qmin, &qmax);
        aa_rx_sg_get_limit_vel(scenegraph, id, &dqmin, &dqmax);
        aa_rx_sg_get_limit_vel(scenegraph, id, &ddqmin, &ddqmax);

        limits->q[i] = AA_MIN(fabs(qmax), fabs(qmin));
        limits->dq[i] = AA_MIN(fabs(qmax), fabs(qmin));
        limits->ddq[i] = AA_MIN(fabs(qmax), fabs(qmin));
    }

    sns_chan_open(&blend_cx.path_in, path_channel_name, NULL);
    blend_cx.follow_cx = &follow_cx;

    // TODO: make these params command line args with these as defaults.
    follow_cx.mode = SNS_MOTOR_MODE_VEL;
    follow_cx.k_p = 0.5;
    follow_cx.frequency = opt_sim_frequency;
    follow_cx.new_traj = false;

    /* Setup Event Handler. */
    struct sns_evhandler handlers[2];
    handlers[0].channel = &blend_cx.path_in;
    handlers[0].context = &blend_cx;
    handlers[0].handler = handle_blend_waypoint;
    handlers[0].ach_options = ACH_O_LAST;

    handlers[1].channel = &follow_cx.state_in->channel;
    handlers[1].context = &follow_cx;
    handlers[1].handler = handle_follow_state;
    handlers[1].ach_options = ACH_O_LAST | ACH_O_WAIT;

    /* Run event loop. */
    enum ach_status r =
        sns_evhandle(handlers, 2,
                     NULL, NULL, NULL,
                     sns_sig_term_default,
                     ACH_EV_O_PERIODIC_TIMEOUT);

    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                "Could not handle events: %s, %s\n",
                ach_result_to_string(r), strerror(errno));

    sns_end();

    return r;
}

enum ach_status handle_blend_waypoint(void *cx_, void *msg_, size_t msg_size)
{
    struct traj_blend_cx *cx = (struct traj_blend_cx *)cx_;
    struct sns_msg_path_dense *msg = (struct sns_msg_path_dense *)msg_;

    /* Turn this message into descritized position/velocity commands. */
    struct aa_ct_pt_list *pt_list = sns_to_amino_path(cx->follow_cx->reg, msg);

    // Blend the path.
    struct aa_ct_seg_list *seg_list =
            aa_ct_tjq_pb_generate(cx->follow_cx->reg, pt_list, cx->limits);

    // Setup the follow context.
    cx->follow_cx->new_traj = true;
    aa_ct_seg_list_destroy(cx->follow_cx->seg_list);
    cx->follow_cx->seg_list = seg_list;
    // When the next motor state message is recieved, following the new
    // trajectory will begin.

    return (ACH_OK);
}

enum ach_status handle_follow_state(void *cx_, void *msg_, size_t msg_size)
{
    struct traj_follow_cx *cx = (struct traj_follow_cx *)cx_;
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state *)msg_;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    size_t n_q = aa_ct_seg_list_n_q(cx->seg_list);

    // Verify the message.
    if (sns_msg_motor_state_check_size(msg, msg_size)) {
        SNS_LOG(LOG_ERR, "Mismatched message size on channel\n");
        return (ACH_OK);
    } else if (msg->header.n != n_q) {
        SNS_LOG(LOG_ERR, "Motor state config size mismatch\n");
        return (ACH_OK);
    } else if (sns_msg_is_expired(&msg->header, &now)) {
        return (ACH_OK);
    }

    double seconds = ((double) now.tv_sec + (double) now.tv_nsec / (double) 1e9L);

    // Verify our context.
    if (cx->seg_list == NULL) {
        // We haven't been sent a waypoint list yet. Just keep waiting.
        return (ACH_OK);
    } else if (cx->new_traj == true) {
        cx->start_time = seconds;
        cx->new_traj = false;
    }

    double reltime = seconds - cx->start_time;
    if (reltime >= aa_ct_seg_list_duration(cx->seg_list)) {
        // We've extended beyond past the end of the trajectory. Do nothing.
        return (ACH_OK);
    }

    struct aa_ct_state *ideal = aa_ct_state_alloc(cx->reg, n_q, 0);
    int r = aa_ct_seg_list_eval(cx->seg_list, ideal, reltime);
    if (r == AA_CT_SEG_OUT) {
        // We've also extended beyond past the end of the trajectory.
        return (ACH_OK);
    }

    // Send the motor reference message.
    struct sns_msg_motor_ref *out_msg = sns_msg_motor_ref_local_alloc(n_q);
    sns_msg_set_time(&out_msg->header, &now, (int64_t)(1e9) / cx->frequency);
    out_msg->mode = cx->mode;
    if (cx->mode == SNS_MOTOR_MODE_POS) {
        for (size_t i = 0; i < n_q; i++) {
            out_msg->u[i] = ideal->q[i];
        }
    } else if (cx->mode == SNS_MOTOR_MODE_VEL) {
        for (size_t i = 0; i < n_q; i++) {
            out_msg->u[i] = ideal->dq[i] - cx->k_p * (msg->X[i].pos - ideal->q[i]);
        }
    } else {
        // Bad motor mode? Write 0's to be safe.
        for (size_t i = 0; i < n_q; i++) {
            out_msg->u[i] = 0;
        }
    }

    sns_msg_motor_ref_put(&cx->ref_out->channel, out_msg);
    aa_mem_region_local_pop(out_msg);

    return (ACH_OK);
}

struct aa_ct_pt_list *sns_to_amino_path(struct aa_mem_region *reg, struct sns_msg_path_dense *path)
{
    struct aa_ct_pt_list *aa_list = aa_ct_pt_list_create(reg);

    int n_steps = path->n_steps;
    size_t n_q = path->n_dof;
    for (int i = 0; i < n_steps; i++)
    {
        struct aa_ct_state *state = aa_ct_state_alloc(reg, n_q, 0);
        AA_MEM_CPY(state->q, path->x + i * n_steps, n_q);
        aa_ct_pt_list_add(aa_list, state);
    }

    return aa_list;
}
