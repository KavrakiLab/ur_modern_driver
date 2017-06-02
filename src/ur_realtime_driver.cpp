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

    struct aa_mem_rlist *names_list = aa_mem_rlist_alloc( aa_mem_region_local_get() );

    const char *opt_chan_state = NULL;
    const char *opt_scene_plugin = NULL;
    const char *opt_scene_name = NULL;
    const double opt_sim_frequecy = 100;
    std::string host;

    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "s:o:i:n:r:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-ksim",
                                     "Copyright (c) 2017, Rice University\n",
                                     "Neil T. Dantam")
                case 's':
                    opt_scene_plugin = optarg;
                    break;
                case 'n':
                    opt_scene_name = optarg;
                    break;
                case 'o':
                    opt_chan_state = optarg;
                    break;
                case 'i':
                    aa_mem_rlist_push_ptr( names_list, optarg );
                    cx.n_ref++;
                    break;
                case 'r':
                    host.assign(optarg);
                    break;
                case '?':   /* help     */
                case 'h':
                    puts( "Usage: sns-ksim -i INPUT_CHANNEL -o OUTPUT_CHANNEL -s SCENE_PLUGIN\n"
                                  "Kinematically simulate a robot.\n"
                                  "\n"
                                  "Options:\n"
                                  "  -o,                       state output channel\n"
                                  "  -i,                       reference input channel\n"
                                  "  -s,                       scenegraph plugin\n"
                                  "  -n,                       scenegraph name\n"
                                  "  -r,                       robot ip address\n"
                                  "  -V,                       Print program version\n"
                                  "  -?,                       display this help and exit\n"
                                  "\n"
                                  "Examples:\n"
                                  "  sns-ksim -o state -i ref -s libmyrobot.so -n myrobot -r 192.168.0.19\n"
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
    /* state channel */
    SNS_REQUIRE( opt_chan_state, "Need output channel");
    SNS_LOG(LOG_INFO, "State Channel: `%s'\n", opt_chan_state);
    sns_chan_open( &cx.state_out, opt_chan_state , NULL );

    /* Scene Plugin */
    SNS_REQUIRE( NULL != opt_scene_plugin, "Need a scene plugin");
    SNS_REQUIRE( NULL != opt_scene_name, "Need a scene name");
    cx.scenegraph = aa_rx_dl_sg(opt_scene_plugin, opt_scene_name, NULL);
    SNS_REQUIRE( NULL != cx.scenegraph, "Could not load scene plugin");
    aa_rx_sg_init(cx.scenegraph);
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);
    cx.q_act = AA_NEW_AR(double,cx.n_q);
    cx.dq_act = AA_NEW_AR(double,cx.n_q);
    cx.q_ref = AA_NEW_AR(double,cx.n_q);
    cx.dq_ref = AA_NEW_AR(double,cx.n_q);

    clock_gettime(ACH_DEFAULT_CLOCK, &cx.t);

    cx.handlers = AA_NEW_AR( struct sns_evhandler, cx.n_ref);
    cx.in = AA_NEW_AR(struct in_cx, cx.n_ref);

    // Initialize arrays
    for( size_t j = cx.n_ref; j; j--) {
        size_t i = j-1;

        const char *name = (const char*)aa_mem_rlist_pop(names_list);
        SNS_LOG(LOG_INFO, "Reference Channel[%lu]: `%s'\n", i, name);

        cx.in[i].name = name;
        cx.in[i].cx = &cx;

        // open channel
        sns_chan_open( &cx.in[i].channel, cx.in[i].name, NULL );

        // init handler
        cx.handlers[i].channel = &cx.in[i].channel;
        cx.handlers[i].context = cx.in+i;
        cx.handlers[i].handler = handle_msg;
        cx.handlers[i].ach_options = ACH_O_LAST;

    }

    SNS_LOG(LOG_INFO, "Simulation Frequency: %.3fkHz\n", opt_sim_frequecy/1e3);
    long period_ns = (long)(1e9 / opt_sim_frequecy);

    cx.period.tv_sec = (time_t)period_ns / (time_t)1e9;
    cx.period.tv_nsec = period_ns % (long)1e9;

    SNS_LOG( LOG_DEBUG, "Simulation Period: %lus + %ldns\n",
             cx.period.tv_sec, cx.period.tv_nsec );


    // Start threads
    pthread_t io_thread;
    if( pthread_create(&io_thread, NULL, io_start, &cx) ) {
        SNS_DIE("Could not create simulation thread: `%s'", strerror(errno));
    }

    // Initialize UrDriver - TODO: may want to add more parameters later as command-line args.
    cx.robot = new UrDriver(cx.rt_msg_cond, cx.msg_cond, host);
    if (!cx.robot->start()) {
        SNS_DIE("Could not start robot driver");
    }

    // Start GUI in main thread
    cx.win = aa_rx_win_default_create ( "sns-ksim", 800, 600 );
    aa_rx_win_set_sg(cx.win, cx.scenegraph);
    sns_start();
    aa_rx_win_run();

    // Stop threads
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
    // Run Loop
    enum ach_status r = sns_evhandle( cx->handlers, cx->n_ref,
                                      &cx->period, io_periodic, cx,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could asdf not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
    // stop window
    if( cx->win ) {
        aa_rx_win_stop(cx->win);
    }
}

enum ach_status handle_msg( void *cx_, void *msg_, size_t frame_size )
{
    struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref *)msg_;
    struct in_cx *cx_in = (struct in_cx*)cx_;
    struct cx *cx = cx_in->cx;

    if( frame_size < sizeof(struct sns_msg_header) ) {
        SNS_LOG(LOG_ERR, "Invalid message size on channel\n");
    } else if( sns_msg_motor_ref_check_size(msg,frame_size) ) {
        SNS_LOG(LOG_ERR, "Mistmatched message size on channel\n");
    } else if( msg->header.n != cx->n_q ) {
        SNS_LOG(LOG_ERR, "Mistmatched element count in reference message\n");
    } else {
        // Message looks OK
        SNS_LOG(LOG_DEBUG, "Got a message on channel %s\n", cx_in->name )
        switch(msg->mode) {
            case SNS_MOTOR_MODE_POS:
                for( size_t i = 0; i < cx->n_q; i ++ ) {
                    cx->q_ref[i] = msg->u[i];
                }
                cx->have_q_ref = 1;
                break;
            case SNS_MOTOR_MODE_VEL:
                for( size_t i = 0; i < cx->n_q; i ++ ) {
                    cx->dq_ref[i] = msg->u[i];
                }
                cx->have_dq_ref = 1;
                break;
            default:
                SNS_LOG(LOG_WARNING, "Unhandled motor mode: `%s'", sns_motor_mode_str(msg->mode));
        }
    }

    return ACH_OK;
}

enum ach_status io_periodic( void *cx_ )
{
    struct cx *cx = (struct cx*)cx_;
    // Run simulation
    command(cx);
    put_state(cx);


    // Update display
    if( cx->win ) aa_rx_win_set_config( cx->win, cx->n_q, cx->q_act );

    // check cancelation
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
    int n_q = (int)cx->n_q;

    // Integrate (euler step)
    // TODO: Replace by commanding to actual robot
//    cblas_daxpy(n_q, dt, cx->dq_act, 1, cx->q_act, 1 );

    std::mutex msg_lock; // Dummy mutex, robot state is already protected.
    std::unique_lock<std::mutex> locker(msg_lock);
    while (!cx->robot->rt_interface_->robot_state_->getControllerUpdated()) {
        cx->rt_msg_cond.wait(locker);
    }

    std::vector<double> pos = cx->robot->rt_interface_->robot_state_->getQActual();
    std::vector<double> vel = cx->robot->rt_interface_->robot_state_->getQdActual();

    // Get currents and torques.
//    std::vector<double> current = cx->robot->rt_interface_->robot_state_->getIActual();
//    std::vector<double> tcp = cx->robot->rt_interface_->robot_state->getTcpForce();

    for (auto i = 0; i < cx->n_q; i++) {
        cx->q_act[i] = pos[i];
        cx->dq_act[i] = vel[i];
    }
    cx->robot->rt_interface_->robot_state_->setControllerUpdated();

    // TODO: Publish robot state here?

    // Set Refs
    if( cx->have_q_ref ) {
        // Set ref pos
        cblas_dcopy( n_q, cx->q_ref, 1, cx->q_act, 1 );
        AA_MEM_ZERO(cx->dq_act, cx->n_q);
        std::vector<double> cmd_pos(cx->q_ref, cx->q_ref + cx->n_q);

        // Command the robot to the reference position.
        cx->robot->servoj(cmd_pos);
    } else if( cx->have_dq_ref ) {
        // Set ref vel
        cblas_dcopy( n_q, cx->dq_ref, 1, cx->dq_act, 1 );

        // Taken from ur_hardware_interface
        double max_vel_change_ = 0.12;

        // Command the robot to the reference velocity.
        cx->robot->setSpeed(cx->dq_ref[0], cx->dq_ref[1], cx->dq_ref[2], cx->dq_ref[3],
                            cx->dq_ref[4], cx->dq_ref[5], max_vel_change_ * 125);
    }

    cx->have_q_ref = 0;
    cx->have_dq_ref = 0;

    return ACH_OK;
}

void put_state( struct cx *cx )
{
    struct sns_msg_motor_state *msg = sns_msg_motor_state_local_alloc((uint32_t)cx->n_q);

    sns_msg_set_time(&msg->header, &cx->t, (int64_t)1e9);
    msg->header.seq = cx->seq++;

    double *pos = sns_msg_motor_state_pos(msg);
    double *vel = sns_msg_motor_state_vel(msg);
    int incpos = (int)sns_msg_motor_state_incpos(msg);
    int incvel = (int)sns_msg_motor_state_incvel(msg);
    int n_q = (int)cx->n_q;

    cblas_dcopy( n_q, cx->q_act, 1, pos, incpos );
    cblas_dcopy( n_q, cx->dq_act, 1, vel, incvel );

    sns_msg_motor_state_put(&cx->state_out, msg);

    aa_mem_region_local_pop(msg);

}
