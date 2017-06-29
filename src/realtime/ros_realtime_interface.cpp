/*
 * ros_realtime_interface.cpp
 * A ros action node that accepts waypoint paths and executes the blended path
 * on the actual robot using the realtime sns, ach, and amino infrastructure.
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

#include <ros/ros.h>
#include <sns.h>
#include <sns/event.h>
#include <sns/path.h>
#include <ach.h>
#include <pthread.h>
#include <poll.h>
#include <ach/experimental.h>

#include <actionlib/server/simple_action_server.h>
#include <ur_modern_driver/FollowWaypointsAction.h>

// Early def.

enum ach_status handle_finished(void *cx_, void *msg_, size_t msg_size);
void* listen_for_finished(void *cx);

class RosRealtimeInterface
{
public:
    ach_channel_t channel_finished;

    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pthread_t finished_thread;
    bool is_finished = false;


    RosRealtimeInterface(std::string name) :
            as_(nh_, name, boost::bind(&RosRealtimeInterface::executeCB, this, _1), false),
            action_name_(name)
    {
        // Open a sns channel. TODO: make the name of the channel a command line argument.
        sns_init();
        sns_chan_open(&channel_path, "follow_path", NULL);
        sns_chan_open(&channel_finished, "path_finished", NULL);
        pthread_mutex_init(&mutex, 0);
        pthread_cond_init(&cond, 0);
        // Setup a handler for the event loop in another thread.
        if (pthread_create(&finished_thread, NULL, listen_for_finished, this) ) {
            SNS_DIE("Could not create a listening thread: '%s'", strerror(errno));
        }

        as_.start();
    }

    ~RosRealtimeInterface()
    {
        sns_end();
        pthread_mutex_destroy(&mutex);
        pthread_cond_destroy(&cond);
        pthread_kill(finished_thread, 9);
    }

    void executeCB(const ur_modern_driver::FollowWaypointsGoalConstPtr &goal)
    {
        ROS_INFO("%s: blending and following path with %zu waypoints",
                action_name_.c_str(), goal->waypoints.points.size());

        size_t n_dof = goal->waypoints.joint_names.size();
        size_t n_steps = goal->waypoints.points.size();

        // Turn the waypoints into an sns_msg_path_dense struct.
        sns_msg_path_dense *path = sns_msg_path_dense_alloc(n_steps, n_dof);
        for (int i = 0; i < n_steps; i++) {
            for (int j = 0; j < n_dof; j++) {
                path->x[i * n_dof + j] = goal->waypoints.points[i].positions[j];
            }
        }

        // Send the struct to the blender.
        struct timespec now;
        clock_gettime(ACH_DEFAULT_CLOCK, &now);
        sns_msg_set_time(&path->header, &now, (int64_t)(1e9)); // 1 sec duration.
        enum ach_status r = ach_put(&channel_path, path,
                sizeof(struct sns_msg_path_dense) - sizeof(sns_real_t) +
                sizeof(sns_real_t) * n_steps * n_dof);

        aa_mem_region_local_pop(path);

        // TODO: what to do when r is weird?
        // TODO: listen to the ach state channel and return those as feedback.
        // TODO: how to preempt this trajectory?

        // Wait until the trajectory is finished, sleeping until then.
        pthread_mutex_lock(&mutex);
        ROS_INFO("Waiting for condition variable to be signaled.");
        pthread_cond_wait(&cond, &mutex);
        is_finished = false;
        pthread_mutex_unlock(&mutex);

        ROS_INFO("Signal variable was signaled, trajectory is finished.");
        result_.result = true;
        as_.setSucceeded(result_);
    }
protected:
    ros::NodeHandle nh_;

    // Nodehandle instance must be created before the action server.
    actionlib::SimpleActionServer<ur_modern_driver::FollowWaypointsAction> as_;
    std::string action_name_;

    ur_modern_driver::FollowWaypointsFeedback feedback_;
    ur_modern_driver::FollowWaypointsResult result_;

    ach_channel_t channel_path;
};

enum ach_status handle_finished(void *cx_, void *msg_, size_t msg_size)
{
    ROS_INFO("Recieved a finished message.");
    RosRealtimeInterface *rri = (RosRealtimeInterface *)cx_;
    pthread_mutex_lock(&rri->mutex);
    rri->is_finished = true;
    pthread_cond_signal(&rri->cond);
    pthread_mutex_unlock(&rri->mutex);
    return ACH_OK;
}

void* listen_for_finished(void *cx)
{
    // Start the event handler.
    RosRealtimeInterface *rri = (RosRealtimeInterface *)cx;

    struct sns_evhandler handlers[1];
    handlers[0].channel = &rri->channel_finished;
    handlers[0].context = rri;
    handlers[0].handler = handle_finished;
    handlers[0].ach_options = ACH_O_LAST;

    enum ach_status r = sns_evhandle(handlers, 1, NULL, NULL, NULL,
            sns_sig_term_default, ACH_EV_O_PERIODIC_TIMEOUT);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_follow_trajectory");

    RosRealtimeInterface interface("realtime_follower");
    ros::spin();

    return 0;
}
