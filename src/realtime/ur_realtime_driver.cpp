/*
 * ur_realtime_driver.cpp
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

#ifndef PACKAGE_VERSION

#define PACKAGE_VERSION "1.0"

#endif

#include "ur_modern_driver/realtime/ur_realtime_driver.h"
#include <amino/math.h>


int print_restrict_count = 0;

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx,1);

    const double opt_sim_frequecy = 250;
    struct sns_motor_channel *last_mc = NULL;
    std::vector<std::string> hosts;
    bool use_window = true;

    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "xy:u:p:r:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("ur_realtime_driver",
                                     "Copyright (c) 2017, Rice University\n",
                                     "William Cannon Lewis")
                case 'x':
                    use_window = false;
                    break;
                case 'y':
                    sns_motor_channel_push( optarg, &cx.state_out );
                    last_mc = cx.state_out;
                    break;
                case 'u':
                    sns_motor_channel_push( optarg, &cx.ref_in );
                    last_mc = cx.ref_in;
                    break;
                case 'p':
                    if( last_mc ) {
                        last_mc->priority = atoi(optarg);
                    } else {
                        SNS_DIE("No channel specified for priority argument");
                    }
                    break;
                case 'r':
                    hosts.push_back(std::string(optarg));
                    break;
                case '?':   /* help     */
                case 'h':
                    puts( "Usage: ur_realtime_driver -u REF_CHANNEL -y STATE_CHANNEL\n"
                                  "Run SNS on the real UR5s.\n"
                                  "\n"
                                  "Options:\n"
                                  "  -x,                       if present, turns off the simulator window.\n"
                                  "  -y <channel>,             state output channel\n"
                                  "  -u <channel>,             reference input channel\n"
                                  "  -p <priority>,            channel priority\n"
                                  "  -r <ip addr>,             robot ip address\n"
                                  "  -V,                       Print program version\n"
                                  "  -?,                       display this help and exit\n"
                                  "\n"
                                  "Environment:\n"
                                  "  SNS_SCENE_PLUGIN          Shared object (plugin) defining the scene\n"
                                  "\n"
                                  "  SNS_SCENE_NAME            Name of the scene within the plugin\n"
                                  "\n"
                                  "  SNS_CHANNEL_MAP_name      Channel remap list for `name'\n"
                                  "\n"
                                  "Examples:\n"
                                  "  ur_realtime_driver -y state -u ref -r 192.168.0.19\n"
                                  "\n"
                                  "Report bugs to <bsw2@rice.edu>"
                    );
                    exit(EXIT_SUCCESS);
                default:
                    SNS_DIE("Unknown Option: `%c'\n", c);
                    break;
            }
        }
    }
    sns_init();

    /* Scene Plugin */
    cx.scenegraph = sns_scene_load();
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);

    /* State */
    sns_motor_state_init(cx.scenegraph,
                         cx.state_out, &cx.state_set,
                         0, NULL);
    clock_gettime(ACH_DEFAULT_CLOCK, &cx.t);

    /* Reference */
    SNS_REQUIRE( cx.ref_in, "Need reference channel");
    {
        size_t n_ref = sns_motor_channel_count(cx.ref_in);
        cx.handlers = AA_NEW_AR( struct sns_evhandler, n_ref );
        sns_motor_ref_init(cx.scenegraph,
                           cx.ref_in, &cx.ref_set,
                           n_ref, cx.handlers);
    }

    printf("Simulation Frequency: %.3fkHz\n", opt_sim_frequecy/1e3);
    long period_ns = (long)(1e9 / opt_sim_frequecy);

    cx.period.tv_sec = (time_t)period_ns / (time_t)1e9;
    cx.period.tv_nsec = period_ns % (long)1e9;

    SNS_LOG( LOG_DEBUG, "Simulation Period: %lus + %ldns\n",
             cx.period.tv_sec, cx.period.tv_nsec );

    cx.last_sent = (double *)malloc(sizeof(double) * cx.n_q);

    // Initialize all robots. There should be a host ip given for each robot to be connected to.
    unsigned int rev_port = 50007; // If rev port is the same, we get an error when binding.
    for (auto host : hosts) {
        auto robot = new UrDriver(cx.rt_msg_cond, cx.msg_cond, host, rev_port);
        rev_port++; 
        if (!robot->start()) {
            SNS_DIE("Could not start robot driver for host %s", host.c_str());
        }

        cx.robots.push_back(robot);
    }

    sns_start();


    /* Start threads */
    if (use_window) {
        pthread_t io_thread;
        if( pthread_create(&io_thread, NULL, io_start, &cx) ) {
            SNS_DIE("Could not create simulation thread: `%s'", strerror(errno));
        }

        /* Start GUI in main thread */
        cx.win = aa_rx_win_default_create ( "ur realtime driver", 800, 600 );
        aa_rx_win_set_sg(cx.win, cx.scenegraph);
        aa_rx_win_run();

        /* Stop threads */
        sns_cx.shutdown = 1;
        if( pthread_join(io_thread, NULL) ) {
            SNS_LOG(LOG_ERR, "Could not join simulation thread: `%s'", strerror(errno));
        }
    } else {
        cx.win = NULL;
        io(&cx);
    }

    sns_end();

    return 0;
}


void* io_start(void *cx) {
    io((struct cx*)cx);
    return NULL;
}

void io(struct cx *cx) {
    /* Run Loop */
    printf("Starting ur driver.\n");
    enum ach_status r = sns_evhandle( cx->handlers, sns_motor_channel_count(cx->ref_in),
                                      &cx->period, io_periodic, cx,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );

    /* stop window */
    if( cx->win ) {
        aa_rx_win_stop(cx->win);
    }
}

enum ach_status io_periodic( void *cx_ )
{
    struct cx *cx = (struct cx*)cx_;

    /* Command to robot */
    command(cx);

    /* Post state */
    struct aa_ct_state *state = sns_motor_state_get(cx->state_set);
    sns_motor_state_put( cx->state_set, &cx->t, (int64_t)2e9 );

    /* Update display */
    if( cx->win ) aa_rx_win_set_config( cx->win, state->n_q, state->q );

    /* check cancelation */
    if( sns_cx.shutdown ) {
        return ACH_CANCELED;
    } else {
        return ACH_OK;
    }
}


enum ach_status command( struct cx *cx )
{
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    double dt = aa_tm_timespec2sec( aa_tm_sub(now, cx->t) );
    cx->t = now;

    struct aa_ct_state *state = sns_motor_state_get(cx->state_set);

    // Record which messages have been received.
    std::vector<bool> have_q_ref(cx->robots.size(), false);
    std::vector<bool> have_dq_ref(cx->robots.size(), false);

    // Command position and velocity arrays.
    std::vector<std::vector<double>> cmd_pos;
    std::vector<std::vector<double>> cmd_vel;

    double max_vel_change = 0.7;

    int robot_idx = 0;
    for (auto robot : cx->robots) {
        // Initialize vectors to hold commanded positions, velocities.
        cmd_pos.push_back(std::vector<double>(UR5_JOINT_N, 0.0));
        cmd_vel.push_back(std::vector<double>(UR5_JOINT_N, 0.0));

        std::mutex msg_lock; // Dummy mutex, robot state is already protected.
        std::unique_lock<std::mutex> locker(msg_lock);
        while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
            cx->rt_msg_cond.wait(locker);
        }

        // Get current position and velocity from the real robot.
        std::vector<double> pos = robot->rt_interface_->robot_state_->getQActual();
        std::vector<double> vel = robot->rt_interface_->robot_state_->getQdActual();

        // Copy over real robot state to controller internals.
        for (auto i = 0; i < pos.size(); i++) {
            *(state->q  + (UR5_GRIPPER_JOINT_N * robot_idx) + i) = pos[i];
            *(state->dq + (UR5_GRIPPER_JOINT_N * robot_idx) + i) = vel[i];
        }
        robot->rt_interface_->robot_state_->setControllerUpdated();
        robot_idx++;
    }

    /* Collect references */
    sns_motor_ref_collate(&cx->t, cx->ref_set);

    /* Process References */
    assert(cx->n_q == cx->ref_set->n_q);

    bool dont_send = true;
    for (size_t i = 0; i < cx->ref_set->n_q; i++) {
        if (!aa_feq(cx->ref_set->u[i], 0.0, 0.0000005) &&
             aa_feq(cx->last_sent[i], 0.0, 0.0000005)) {
            dont_send = false;
        } else if (!aa_feq(cx->last_sent[i], 0.0, 0.0000005)) {
            dont_send = false;
        }
    }
    if (dont_send) {
        if (print_restrict_count++ % 1000 == 1) {
	    fprintf(stdout, "Two motor commands equal 0. Not sending next command, calling stop instead.\n");  
        }
        for (size_t i = 0; i < cx->robots.size(); i++) {
            cx->robots[i]->stopTraj();
        }
        return ACH_OK;
    }

    for( size_t i = 0; i < cx->ref_set->n_q; i ++ ) {
        struct sns_motor_ref_meta *m = cx->ref_set->meta+i;
        double u = cx->ref_set->u[i];
        double *q = state->q+i;
        double *dq = state->dq+i;


        /**
         * Since we iterate over all joints in the scenegraph, we must
         * split the appropriate position and velocity references to
         * command to the appropriate connected robot.
         */
        size_t robot_num = i / UR5_GRIPPER_JOINT_N;
        size_t joint_num = i % UR5_GRIPPER_JOINT_N;
        // Out of robots or out of joints (we can't control the grippers from here). 
	if (robot_num + 1 > cx->robots.size() || joint_num > UR5_JOINT_N)
            continue;
        if( aa_tm_cmp(now,m->expiration) < 0 ) {
            switch(m->mode) {
                case SNS_MOTOR_MODE_POS:
                    have_q_ref[robot_num] = true;
                    cmd_pos[robot_num][joint_num] = u;
                    cmd_vel[robot_num][joint_num] = 0.0;
                    break;
                case SNS_MOTOR_MODE_VEL:
                    have_dq_ref[robot_num] = true;
                    cmd_vel[robot_num][joint_num] = u;
                    break;
                case SNS_MOTOR_MODE_HALT:
                    have_dq_ref[robot_num] = true;
                    cmd_vel[robot_num][joint_num] = 0;
                    break;
                default:
                    SNS_LOG(LOG_WARNING, "Unhandled mode for motor %lu", i );
            }
        } else {
            /* reference has expired */
            *dq = 0;
        }
    }

    for (size_t i = 0; i < cx->ref_set->n_q; i++) {
        cx->last_sent[i] = cx->ref_set->u[i];
    }

    // Here we actually send commands to the real robot using the UrDriver class.
    for (size_t i = 0; i != cx->robots.size(); i++) {
        if (have_q_ref[i] && have_dq_ref[i]) {
            // If we have both a position and velocity reference, send position, then velocity
            fprintf(stdout, "We have both a position and a velocity reference, this is weird.\n");
            cx->robots[i]->servoj(cmd_pos[i]);
            cx->robots[i]->setSpeed(cmd_vel[i][0], cmd_vel[i][1], cmd_vel[i][2],
                                    cmd_vel[i][3], cmd_vel[i][4], cmd_vel[i][5], max_vel_change);
        } else if (have_q_ref[i]) {
            // We only send a position reference to the robot.
            cx->robots[i]->servoj(cmd_pos[i]);
        } else if (have_dq_ref[i]) {
            // We only send a velocity reference to the robot.
            cx->robots[i]->setSpeed(cmd_vel[i][0], cmd_vel[i][1], cmd_vel[i][2],
                                    cmd_vel[i][3], cmd_vel[i][4], cmd_vel[i][5], max_vel_change);
        }
    }

    return ACH_OK;
}
