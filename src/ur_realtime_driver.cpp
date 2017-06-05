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

#include "ur_modern_driver/ur_realtime_driver.h"

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx,1);

    const double opt_sim_frequecy = 100;
    struct sns_motor_channel *last_mc = NULL;
    std::vector<std::string> hosts;

    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "y:u:p:r:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-ksim",
                                     "Copyright (c) 2017, Rice University\n",
                                     "Neil T. Dantam")
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
                    puts( "Usage: sns-ksim -u REF_CHANNEL -y STATE_CHANNEL\n"
                                  "Kinematically simulate a robot.\n"
                                  "\n"
                                  "Options:\n"
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
                                  "  sns-ksim -y state -u ref -r 192.168.0.19\n"
                                  "\n"
                                  "Report bugs to <ntd@rice.edu>"
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

    SNS_LOG(LOG_INFO, "Simulation Frequency: %.3fkHz\n", opt_sim_frequecy/1e3);
    long period_ns = (long)(1e9 / opt_sim_frequecy);

    cx.period.tv_sec = (time_t)period_ns / (time_t)1e9;
    cx.period.tv_nsec = period_ns % (long)1e9;

    SNS_LOG( LOG_DEBUG, "Simulation Period: %lus + %ldns\n",
             cx.period.tv_sec, cx.period.tv_nsec );


    /* Start threads */
    pthread_t io_thread;
    if( pthread_create(&io_thread, NULL, io_start, &cx) ) {
        SNS_DIE("Could not create simulation thread: `%s'", strerror(errno));
    }

    // Initialize all robots. There should be a host ip given for each robot to be connected to.
    for (auto host : hosts) {
        auto robot = new UrDriver(cx.rt_msg_cond, cx.msg_cond, host);

        if (robot->start()) {
            SNS_DIE("Could not start robot driver for host %s", host.c_str());
        }

        cx.robots.push_back(robot);
    }

    /* Start GUI in main thread */
    cx.win = aa_rx_win_default_create ( "sns-ksim", 800, 600 );
    aa_rx_win_set_sg(cx.win, cx.scenegraph);
    sns_start();
    aa_rx_win_run();

    /* Stop threads */
    sns_cx.shutdown = 1;
    if( pthread_join(io_thread, NULL) ) {
        SNS_LOG(LOG_ERR, "Could not join simulation thread: `%s'", strerror(errno));
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
    enum ach_status r = sns_evhandle( cx->handlers, sns_motor_channel_count(cx->ref_in),
                                      &cx->period, io_periodic, cx,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could asdf not handle events: %s, %s\n",
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
    sns_motor_state_put( cx->state_set, &cx->t, (int64_t)1e9 );

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

    double max_vel_change = 0.12;

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
        for (auto i = 0; i < cx->n_q; i++) {
            *(state->q + i) = pos[i];
            *(state->dq + i) = vel[i];
        }
       robot->rt_interface_->robot_state_->setControllerUpdated();
    }

    /* Collect references */
    sns_motor_ref_collate(&cx->t, cx->ref_set);

    /* Process References */
    assert(cx->n_q == cx->ref_set->n_q );
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
        size_t robot_num = i / UR5_JOINT_N;
        size_t joint_num = i % UR5_JOINT_N;
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

    // Here we actually send commands to the real robot using the UrDriver class.
    for (size_t i = 0; i != cx->robots.size(); i++) {
        if (have_q_ref[i] && have_dq_ref[i]) {
            // If we have both a position and velocity reference, send position, then velocity
            cx->robots[i]->servoj(cmd_pos[i]);
            cx->robots[i]->setSpeed(cmd_vel[i][0], cmd_vel[i][1], cmd_vel[i][2],
                    cmd_vel[i][3], cmd_vel[i][4], cmd_vel[i][5], max_vel_change * 125);
        } else if (have_q_ref[i]) {
            // We only send a position reference to the robot.
            cx->robots[i]->servoj(cmd_pos[i]);
        } else if (have_dq_ref[i]) {
            // We only send a velocity reference to the robot.
            cx->robots[i]->setSpeed(cmd_vel[i][0], cmd_vel[i][1], cmd_vel[i][2],
                                    cmd_vel[i][3], cmd_vel[i][4], cmd_vel[i][5], max_vel_change * 125);
        }
    }

    return ACH_OK;
}