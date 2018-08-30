/*
 * ur_driver.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
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

#include "ur_modern_driver/ur_driver.h"
#include "ur_modern_driver/ur_hardware_interface.h"
#include "ur_modern_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// A value used as an additional macro for result_.error_code,
// which is set to this upon receiving a request, and only changed when
// all robots finish successfully, or one fails.
#define WAITING 2

/**
 * Wraps N robots.
 */
class RosWrapperN
{
protected:
    /// Vectors of things directly related to the robot.
    std::vector<UrDriver *> robots_;
    std::vector<ros_control_ur::UrHardwareInterfacePtr> hardware_interfaces_;
    std::vector<std::shared_ptr<controller_manager::ControllerManager> > controller_managers_;

    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    ros::NodeHandle nh_;

    /// Action Server related members.
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
    // TODO(brycew): on the original ThomasTimm repo, PR 101 adds some mutexes to protect has_goal_.
    // Since this gets even more complicated with a vector, adapt these changes to this code.
    std::vector<bool> has_goal_; // Whether each robot has a goal set/ has already reached that goal
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;

    ros::Subscriber speed_sub_;
    ros::Subscriber urscript_sub_;
    ros::Subscriber force_torque_sub_;
    ros::ServiceServer io_srv_;
    ros::ServiceServer payload_srv_;

    // Threads.
    std::thread *rt_publish_thread_; // RT = Real time

    std::thread *mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    std::vector<double> joint_offsets_; // Adjusts the positions of joint i by joint_offsets_[i]
    std::vector<std::string> base_frames_;
    std::vector<std::string> tool_frames_;
    bool use_ros_control_;
    std::thread *ros_control_thread_;
    std::vector<double> Fx_list;
    std::vector<double> Fy_list;
    std::vector<double> Fz_list;
    double max_force_dev_;
    int force_average_count_;

public:
    RosWrapperN(std::vector<std::string> hosts, int reverse_port_start, std::vector<std::string> prefixes)
      : as_(nh_, "follow_joint_trajectory", boost::bind(&RosWrapperN::goalCB, this, _1),
            boost::bind(&RosWrapperN::cancelCB, this, _1), false)
      , io_flag_delay_(0.05)
      , joint_offsets_(hosts.size() * 6, 0.0)
    {
        int reverse_port = reverse_port_start;
        for (std::string host : hosts)
        {
            // TODO: look at what servoj time and safety_count steps (0.03 and 300 in this,
            // but defaults in realtime) should be.
            auto robot = new UrDriver(rt_msg_cond_, msg_cond_, host, reverse_port);
            robots_.push_back(robot);
            reverse_port += 1;
        }

        if (prefixes.size() == 0)
            // This happens if we are only running one robot and don't want to prefix.
            prefixes.push_back("");

        char buf[256];

        for (size_t i = 0; i < robots_.size(); i++)
        {
            std::string joint_prefix = prefixes[i];
            std::vector<std::string> joint_names;
            if (joint_prefix.length() > 0)
            {
                sprintf(buf, "Setting prefix to %s", joint_prefix.c_str());
                print_info(buf);
            }
            joint_names.push_back(joint_prefix + "shoulder_pan_joint");
            joint_names.push_back(joint_prefix + "shoulder_lift_joint");
            joint_names.push_back(joint_prefix + "elbow_joint");
            joint_names.push_back(joint_prefix + "wrist_1_joint");
            joint_names.push_back(joint_prefix + "wrist_2_joint");
            joint_names.push_back(joint_prefix + "wrist_3_joint");
            robots_[i]->setJointNames(joint_names);
        }

        use_ros_control_ = false;
        ros::param::get("~use_ros_control", use_ros_control_);
        if (use_ros_control_)
        {
            print_warning("ROS Control not yet implemented for multiple robots. ROS Control will not be used.");
            use_ros_control_ = false;
        }

        if (use_ros_control_)
        {
            for (auto robot : robots_)
            {
                auto hardware_interface = std::make_shared<ros_control_ur::UrHardwareInterface>(nh_, robot);
                auto controller_manager =
                    std::make_shared<controller_manager::ControllerManager>(hardware_interface.get(), nh_);
                controller_managers_.push_back(controller_manager);
                double max_vel_change = 0.12;  // equivalent of an acceleration of 15 rad/sec^2
                if (ros::param::get("~max_acceleration", max_vel_change))
                {
                    // TODO: why are we dividing by 125?
                    max_vel_change = max_vel_change / 125;
                }
                sprintf(buf, "Max acceleration set to: %f [rad/sec²]", max_vel_change * 125);
                print_debug(buf);
                hardware_interface->setMaxVelChange(max_vel_change);
                hardware_interfaces_.push_back(hardware_interface);
            }
        }
        // Using a very high value in order to not limit execution of trajectories being sent from MoveIt!
        max_velocity_ = 10.;
        if (ros::param::get("~max_velocity", max_velocity_))
        {
            sprintf(buf, "Max velocity accepted by ur_driver: %f [rad/s]", max_velocity_);
            print_debug(buf);
        }

        /******************************* Setting params of Robot. ***************************/
        // Bounds for SetPayload service
        // Using a very conservative value as it should be set through the parameter server
        double min_payload = 0.;
        double max_payload = 1.;
        if (ros::param::get("~min_payload", min_payload))
        {
            sprintf(buf, "Min payload set to: %f [kg]", min_payload);
            print_debug(buf);
        }
        else
        {
            sprintf(buf, "Min payload should be set via parameter server: default at %f [kg]", min_payload);
            print_warning(buf);
        }

        if (ros::param::get("~max_payload", max_payload))
        {
            sprintf(buf, "Max payload set to: %f [kg]", max_payload);
            print_debug(buf);
        }
        else
        {
            sprintf(buf, "Max payload should be set via parameter server: default at %f [kg]", max_payload);
            print_warning(buf);
        }


        double servoj_time = 0.008;
        if (ros::param::get("~servoj_time", servoj_time))
        {
            sprintf(buf, "Servoj_time set to: %f [sec]", servoj_time);
            print_debug(buf);
        }

        double servoj_lookahead_time = 0.03;
        if (ros::param::get("~servoj_lookahead_time", servoj_lookahead_time))
        {
            sprintf(buf, "Servoj_lookahead_time set to: %f [sec]", servoj_lookahead_time);
            print_debug(buf);
        }

        double servoj_gain = 300.;
        if (ros::param::get("~servoj_gain", servoj_gain))
        {
            sprintf(buf, "Servoj_gain set to: %f [sec]", servoj_gain);
            print_debug(buf);
        }

        for (auto robot : robots_)
        {
            robot->setMinPayload(min_payload);
            robot->setMaxPayload(max_payload);
            robot->setServojTime(servoj_time);
            robot->setServojLookahead(servoj_lookahead_time);
            robot->setServojGain(servoj_gain);
        }

        // Base and tool frames
        for (size_t i = 0; i < prefixes.size(); i++)
        {
            base_frames_.push_back(prefixes[i] + "base_link");
            tool_frames_.push_back(prefixes[i] + "tool0_controller");
        }
        ros::param::get("~base_frame", base_frames_);
        ros::param::get("~tool_frame", tool_frames_);

        bool all_bots_started = true;
        for (auto robot : robots_)
        {
            all_bots_started = all_bots_started && robot->start();
        }
        if (all_bots_started)
        {
            if (use_ros_control_)
            {
                // TODO(Someone Else): launch one per robot?
                //ros_control_thread_ = new std::thread(boost::bind(&RosWrapperN::rosControlLoop, this));
                //print_debug("The control thread for this driver has been started");
                print_warning("Not using ROS control, hasn't been implemented yet for multiple URs. Starting the system as normal.");
            }
            //else
            {
                // start actionserver
                for (int i = 0; i < robots_.size(); i++)
                {
                    has_goal_.push_back(false);
                }
                as_.start();

                // subscribe to the data topic of interest
                rt_publish_thread_ = new std::thread(boost::bind(&RosWrapperN::publishRTMsg, this));
                print_debug("The action server for this driver has been started");
            }
            mb_publish_thread_ = new std::thread(boost::bind(&RosWrapperN::publishMbMsg, this));
            speed_sub_ = nh_.subscribe("ur_driver/joint_speed", 1, &RosWrapperN::speedInterface, this);
            // TODO(brycew): Make a UR script for each robot available.
            for (size_t i = 0; i < robots_.size(); i++)
            {
                std::string topic_name = "ur_driver/URScript" + std::to_string(i);
                urscript_sub_ = nh_.subscribe<std_msgs::String>(topic_name, 1, boost::bind(&RosWrapperN::urscriptInterface, this, i, _1));
            }

            if (ros::param::get("~max_force_dev", max_force_dev_))
            {
                sprintf(buf, "max sensor deviation set to: %f [mN]", max_force_dev_);
                print_debug(buf);
            }

            if (ros::param::get("~force_average_count", force_average_count_))
            {
                sprintf(buf, "force average count set to: %d ", force_average_count_);
                print_debug(buf);
            }

            force_torque_sub_ =
                nh_.subscribe("robotiq_force_torque_wrench", 100, &RosWrapperN::force_torque_sub, this);

            io_srv_ = nh_.advertiseService("ur_driver/set_io", &RosWrapperN::setIO, this);
            payload_srv_ = nh_.advertiseService("ur_driver/set_payload", &RosWrapperN::setPayload, this);
        }
    }

    void halt()
    {
        for (auto robot : robots_)
            robot->halt();
        rt_publish_thread_->join();
    }

private:
    /**
     * Gets if any robots are still executing paths from the action server.
     * Just a simple OR union over the has_goal_ vector.
     */
    bool any_executing()
    {
        return std::accumulate(has_goal_.begin(), has_goal_.end(),
                               false, [](bool a, bool b) { return a || b; });
    }

    void stop_all_robots()
    {
        for (size_t i = 0; i < robots_.size(); i++)
        {
            robots_[i]->stopTraj();
            has_goal_[i] = false;
        }
    }

    // TODO(brycew): Put this back to a single thread, rewrite to:
    // * Take the full trajectory here.
    // * parabolic-blend (? or something similar)
    // * move doTraj up here: 
    void trajThread(std::vector<double> timestamps, std::vector<std::vector<double>> positions,
                    std::vector<std::vector<double>> velocities, size_t robot_idx)
    {
        robots_[robot_idx]->doTraj(timestamps, positions, velocities);

        // Gets the number of remaining robots that need to complete goals.
        bool remaining = std::accumulate(has_goal_.begin(), has_goal_.end(),
                                   0, [](int a, bool b) { return a + (b) ? 1 : 0; });
        if (remaining == 1 && has_goal_[robot_idx])
        {
            // If this is the last robot to reach it's goal
            // TODO(brycew): do we need to check if result_ is already failing? What do we do if that is the case?
            result_.error_code = result_.SUCCESSFUL;
            goal_handle_.setSucceeded(result_);
        }
        has_goal_[robot_idx] = false;
    }

    void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
    {
        std::string buf;
        print_info("on_goal");
        for (auto robot : robots_)
        {
            if (!robot->sec_interface_->robot_state_->isReady())
            {
                result_.error_code = -100;  // nothing is defined for this...?

                if (!robot->sec_interface_->robot_state_->isPowerOnRobot())
                {
                    result_.error_string = "Cannot accept new trajectories: Robot arm is not powered on";
                    gh.setRejected(result_, result_.error_string);
                    print_error(result_.error_string);
                    return;
                }
                if (!robot->sec_interface_->robot_state_->isRealRobotEnabled())
                {
                    result_.error_string = "Cannot accept new trajectories: Robot is not enabled";
                    gh.setRejected(result_, result_.error_string);
                    print_error(result_.error_string);
                    return;
                }
                result_.error_string = "Cannot accept new trajectories. (Debug: Robot mode is " +
                    std::to_string(robot->sec_interface_->robot_state_->getRobotMode()) + ")";
                gh.setRejected(result_, result_.error_string);
                print_error(result_.error_string);
                return;
            }
            if (robot->sec_interface_->robot_state_->isEmergencyStopped())
            {
                result_.error_code = -100;  // nothing is defined for this...?
                result_.error_string = "Cannot accept new trajectories: Robot is emergency stopped";
                gh.setRejected(result_, result_.error_string);
                print_error(result_.error_string);
                return;
            }
            if (robot->sec_interface_->robot_state_->isProtectiveStopped())
            {
                result_.error_code = -100;  // nothing is defined for this...?
                result_.error_string = "Cannot accept new trajectories: Robot is protective stopped";
                gh.setRejected(result_, result_.error_string);
                print_error(result_.error_string);
                return;
            }
        }

        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *gh.getGoal();  // make a copy that we can modify

        if (any_executing())
        {
            print_warning("Received new goal while still executing previous trajectory. Canceling previous "
                          "trajectory");
            stop_all_robots();
            result_.error_code = -100;  // nothing is defined for this...?
            result_.error_string = "Received another trajectory";
            goal_handle_.setAborted(result_, result_.error_string);
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        goal_handle_ = gh;
        if (!validateJointNames())
        {
            std::string outp_joint_names = "";
            for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++)
            {
                outp_joint_names += goal.trajectory.joint_names[i] + " ";
            }
            result_.error_code = result_.INVALID_JOINTS;
            result_.error_string = "Received a goal with incorrect joint names: " + outp_joint_names;
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }
        if (!has_positions())
        {
            result_.error_code = result_.INVALID_GOAL;
            result_.error_string = "Received a goal without positions";
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }

        if (!has_velocities())
        {
            result_.error_code = result_.INVALID_GOAL;
            result_.error_string = "Received a goal without velocities";
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }

        if (!traj_is_finite())
        {
            result_.error_string = "Received a goal with infinities or NaNs";
            result_.error_code = result_.INVALID_GOAL;
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }

        if (!has_limited_velocities())
        {
            result_.error_code = result_.INVALID_GOAL;
            result_.error_string =
                "Received a goal with velocities that are higher than " + std::to_string(max_velocity_);
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }

        reorder_traj_joints(goal.trajectory);

        if (!start_positions_match(goal.trajectory, 0.01))
        {
            result_.error_code = result_.INVALID_GOAL;
            result_.error_string = "Goal start doesn't match current pose";
            gh.setRejected(result_, result_.error_string);
            print_error(result_.error_string);
            return;
        }

        // TODO(brycew): split the trajectory into the separate sections for each robot here.
        // Still precompute outside of the loop that spawns threads so the threads can be started at the same time.
        std::vector<double> timestamps;
        std::vector<std::vector<std::vector<double>>> positions(robots_.size()), velocities(robots_.size()); // For each robot, for each point, for each joint.
        if (goal.trajectory.points[0].time_from_start.toSec() != 0.)
        {
            print_warning("Trajectory's first point should be the current position, with time_from_start set "
                          "to 0.0 - Inserting point in malformed trajectory");
            timestamps.push_back(0.0);
            for (size_t i = 0; i < robots_.size(); i++)
            {
                auto robot = robots_[i];
                std::vector<double> start_position = robot->rt_interface_->robot_state_->getQActual();
                std::vector<double> start_velocity = robot->rt_interface_->robot_state_->getQdActual();
                positions[i].push_back(start_position);
                velocities[i].push_back(start_velocity);
            }
        }
        for (unsigned int i = 0; i < goal.trajectory.points.size(); i++)
        {
            timestamps.push_back(goal.trajectory.points[i].time_from_start.toSec());
            size_t traj_offset = 0;
            for (size_t j = 0; j < robots_.size(); j++)
            {
                auto point = goal.trajectory.points[i];
                std::vector<double> split_positions(point.positions.begin() + traj_offset, point.positions.end() + traj_offset + 6);
                std::vector<double> split_velocities(point.velocities.begin() + traj_offset, point.velocities.end() + traj_offset + 6);
                positions[j].push_back(split_positions);
                velocities[j].push_back(split_velocities);
                traj_offset += 6;
            }
        }

        goal_handle_.setAccepted();
        for (size_t i = 0; i < robots_.size(); i++)
        {
            has_goal_[i] = true;
            const std::vector<std::vector<double> > &robot_i_positions = positions[i];
            const std::vector<std::vector<double> > &robot_i_velocities = velocities[i];
            std::thread(&RosWrapperN::trajThread, this, timestamps, robot_i_positions, robot_i_velocities, i).detach();
        }
    }

    void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
    {
        // set the action state to preempted
        print_info("on_cancel");
        if (any_executing())
        {
            if (gh == goal_handle_)
            {
                stop_all_robots();
            }
        }
        result_.error_code = -100;  // nothing is defined for this...?
        result_.error_string = "Goal cancelled by client";
        gh.setCanceled(result_);
    }

    /**
     * Change for double branch: Just sets the IO for all of the robots controlled.
     */
    bool setIO(ur_msgs::SetIORequest &req, ur_msgs::SetIOResponse &resp)
    {
        resp.success = true;
        for (auto robot : robots_)
        {
            // if (req.fun == ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT) {
            if (req.fun == 1)
            {
                robot->setDigitalOut(req.pin, req.state > 0.0 ? true : false);
            }
            else if (req.fun == 2)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_FLAG) {
                robot->setFlag(req.pin, req.state > 0.0 ? true : false);
                // According to urdriver.py, set_flag will fail if called to rapidly in succession
                ros::Duration(io_flag_delay_).sleep();
            }
            else if (req.fun == 3)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_ANALOG_OUT) {
                robot->setAnalogOut(req.pin, req.state);
            }
            else if (req.fun == 4)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_TOOL_VOLTAGE) {
                robot->setToolVoltage((int)req.state);
            }
            else
            {
                resp.success = false;
                break;
            }
        }
        return resp.success;
    }

    /**
     * Change for double branch: Just set the payload for all of the robots controlled.
     */
    bool setPayload(ur_msgs::SetPayloadRequest &req, ur_msgs::SetPayloadResponse &resp)
    {
        for (auto robot : robots_)
        {
            // Note(brycew): This code intentionally returns true in both cases, it's the same in ur_ros_wrapper.
            // No clear explaination of why in the commit log.
            // robot.setPayload() only fails if the requested val is out of the range set in the constructor.
            if (robot->setPayload(req.payload))
                resp.success = true;
            else
                resp.success = true;
        }
        return resp.success;
    }

    bool validateJointNames()
    {
        std::vector<std::string> actual_joint_names;
        actual_joint_names.reserve(robots_.size() * robots_[0]->getJointNames().size());
        for (auto robot : robots_)
        {
            std::vector<std::string> joints = robot->getJointNames();
            actual_joint_names.insert(actual_joint_names.end(), joints.begin(), joints.end());
        }
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *goal_handle_.getGoal();
        if (goal.trajectory.joint_names.size() != actual_joint_names.size())
            return false;

        for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++)
        {
            unsigned int j;
            for (j = 0; j < actual_joint_names.size(); j++)
            {
                if (goal.trajectory.joint_names[i] == actual_joint_names[j])
                    break;
            }
            if (goal.trajectory.joint_names[i] == actual_joint_names[j])
            {
                actual_joint_names.erase(actual_joint_names.begin() + j);
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    void reorder_traj_joints(trajectory_msgs::JointTrajectory &traj)
    {
        /* Reorders trajectory - destructive */
        std::vector<std::string> actual_joint_names;
        actual_joint_names.reserve(robots_.size() * robots_[0]->getJointNames().size());
        for (auto robot : robots_)
        {
            std::vector<std::string> joints = robot->getJointNames();
            actual_joint_names.insert(actual_joint_names.end(), joints.begin(), joints.end());
        }
        std::vector<unsigned int> mapping;
        mapping.resize(actual_joint_names.size(), actual_joint_names.size());
        for (unsigned int i = 0; i < traj.joint_names.size(); i++)
        {
            for (unsigned int j = 0; j < actual_joint_names.size(); j++)
            {
                if (traj.joint_names[i] == actual_joint_names[j])
                    mapping[j] = i;
            }
        }
        traj.joint_names = actual_joint_names;
        std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
        for (unsigned int i = 0; i < traj.points.size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint new_point;
            for (unsigned int j = 0; j < traj.points[i].positions.size(); j++)
            {
                new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
                new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
                if (traj.points[i].accelerations.size() != 0)
                    new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
            }
            new_point.time_from_start = traj.points[i].time_from_start;
            new_traj.push_back(new_point);
        }
        traj.points = new_traj;
    }

    bool has_velocities()
    {
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *goal_handle_.getGoal();
        if (goal.trajectory.points.size() == 0)
            return false;
        for (unsigned int i = 0; i < goal.trajectory.points.size(); i++)
        {
            if (goal.trajectory.points[i].positions.size() != goal.trajectory.points[i].velocities.size())
                return false;
        }
        return true;
    }

    bool has_positions()
    {
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *goal_handle_.getGoal();
        if (goal.trajectory.points.size() == 0)
            return false;
        for (unsigned int i = 0; i < goal.trajectory.points.size(); i++)
        {
            if (goal.trajectory.points[i].positions.size() != goal.trajectory.joint_names.size())
                return false;
        }
        return true;
    }

    bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps)
    {
        size_t traj_offset = 0;
        for (auto robot : robots_)
        {
            std::vector<double> qActual = robot->rt_interface_->robot_state_->getQActual();
            for (size_t i = 0; i < qActual.size(); i++)
            {
                if (fabs(traj.points[0].positions[traj_offset + i] - qActual[i]) > eps)
                    return false;
            }
            traj_offset += qActual.size();
        }
        return true;
    }

    bool has_limited_velocities()
    {
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *goal_handle_.getGoal();
        for (unsigned int i = 0; i < goal.trajectory.points.size(); i++)
        {
            for (unsigned int j = 0; j < goal.trajectory.points[i].velocities.size(); j++)
            {
                if (fabs(goal.trajectory.points[i].velocities[j]) > max_velocity_)
                    return false;
            }
        }
        return true;
    }

    bool traj_is_finite()
    {
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
            *goal_handle_.getGoal();
        for (unsigned int i = 0; i < goal.trajectory.points.size(); i++)
        {
            for (unsigned int j = 0; j < goal.trajectory.points[i].velocities.size(); j++)
            {
                if (!std::isfinite(goal.trajectory.points[i].positions[j]))
                    return false;
                if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
                    return false;
            }
        }
        return true;
    }

    /**
     * As we can set the speed of the robot using a JointTrajectory message,
     * it makes more sense to have this message be the same size as the Trajectories
     * executed.
     */
    void speedInterface(const trajectory_msgs::JointTrajectory::Ptr &msg)
    {
        size_t traj_offset = 0;
        // Always a multiple of 6, we have to have 6 speeds for each robot.
        if (msg->points[0].velocities.size() == 6 * robots_.size())
        {
            for (auto robot : robots_)
            {
                double acc = 100;
                if (msg->points[0].accelerations.size() > 0)
                    acc = *std::max_element(msg->points[0].accelerations.begin() + traj_offset,
                                            msg->points[0].accelerations.end() + robot->getJointNames().size());
                std::vector<double> velocities = msg->points[0].velocities;
                robot->setSpeed(velocities[traj_offset + 0], velocities[traj_offset + 1],
                                velocities[traj_offset + 2], velocities[traj_offset + 3],
                                velocities[traj_offset + 4], velocities[traj_offset + 5], acc);
                traj_offset += 6;
            }
        }
    }

    // TODO(brycew): best way would be to make a separate server part for each UR, so we'd get an index in this call.
    void urscriptInterface(size_t idx, const std_msgs::String::ConstPtr &msg)
    {
        robots_[idx]->rt_interface_->addCommandToQueue(msg->data);
    }

    void force_torque_sub(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        bool estop;
        char buf[256];
        estop = false;
        bool start = false;
        double Fx_avg = 0;
        double Fy_avg = 0;
        double Fz_avg = 0;
        double total_avg = 0;
        double total_current = 0;
        this->Fx_list.push_back(msg->wrench.force.x);
        this->Fy_list.push_back(msg->wrench.force.y);
        this->Fz_list.push_back(msg->wrench.force.z);
        if (this->Fx_list.size() > this->force_average_count_)
        {
            this->Fx_list.erase(this->Fx_list.begin());
            Fx_avg = RosWrapperN::average_vector(this->Fx_list);
            start = true;
        }
        if (this->Fy_list.size() > this->force_average_count_)
        {
            this->Fy_list.erase(this->Fy_list.begin());
            Fy_avg = RosWrapperN::average_vector(this->Fy_list);
        }
        if (this->Fz_list.size() > this->force_average_count_)
        {
            this->Fz_list.erase(this->Fz_list.begin());
            Fz_avg = RosWrapperN::average_vector(this->Fz_list);
        }

        total_avg = std::sqrt(Fx_avg * Fx_avg + Fy_avg * Fy_avg + Fz_avg * Fz_avg);
        total_current =
            std::sqrt(msg->wrench.force.x * msg->wrench.force.x + msg->wrench.force.y * msg->wrench.force.y +
                      msg->wrench.force.z * msg->wrench.force.z);
        if (((total_current < total_avg - this->max_force_dev_) ||
             (total_current > total_avg + this->max_force_dev_)) &&
            start)
        {
            estop = true;
        }
        sprintf(buf, "total_max = %f \nmin total_min = %f", total_avg + this->max_force_dev_,
                total_avg - this->max_force_dev_);
        print_debug(buf);
        sprintf(buf, "Force avgs \nFx = %f \nFy = %f, \nFz=%f \ntotal=%f", Fx_avg, Fy_avg, Fz_avg, total_avg);
        print_debug(buf);
        sprintf(buf, "this->max_force_dev_ = %f", this->max_force_dev_);
        print_debug(buf);
        if (estop == false)
        {
            sprintf(buf, "system is safe");
            print_debug(buf);
        }
        else
        {
            sprintf(buf, "Force avgs \nFx = %f \nFy = %f, \nFz=%f \ntotal=%f", Fx_avg, Fy_avg, Fz_avg,
                    total_avg);
            print_info(buf);
            sprintf(buf, "MAXIMUM REACHED, TOTAL VALUE = %f", total_current);
            print_info(buf);
            stop_all_robots();
            // RosWrapperN::halt(); //stop traj will stop motion, this will kill entire driver.
        }
    }

    double average_vector(std::vector<double> vector)
    {
        double average;
        double total;
        int length;
        length = vector.size();
        total = 0;

        for (int i = 0; i < length; i++)
        {
            total += vector[i];
        }
        average = (total / length);
        return average;
    }

    // TODO(Someone Else): Handle ros_control for multiple URs.
    // No one is using it at the moment, so it's getting commented out.
    /*
    void rosControlLoop()
    {
        ros::Duration elapsed_time;
        struct timespec last_time, current_time;
        static const double BILLION = 1000000000.0;

        realtime_tools::RealtimePublisher<tf::tfMessage> tf_pub(nh_, "/tf", 1);
        geometry_msgs::TransformStamped tool_transform;
        tool_transform.header.frame_id = base_frame_;
        tool_transform.child_frame_id = tool_frame_;
        tf_pub.msg_.transforms.push_back(tool_transform);

        realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> tool_vel_pub(nh_, "tool_velocity", 1);
        tool_vel_pub.msg_.header.frame_id = base_frame_;

        clock_gettime(CLOCK_MONOTONIC, &last_time);
        while (ros::ok())
        {
            std::mutex msg_lock;  // The values are locked for reading in the class, so just use a dummy mutex
            std::unique_lock<std::mutex> locker(msg_lock);
            while (!robot_.rt_interface_->robot_state_->getControllerUpdated())
            {
                rt_msg_cond_.wait(locker);
            }
            // Input
            hardware_interface_->read();
            robot_.rt_interface_->robot_state_->setControllerUpdated();

            // Control
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec +
                                         (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
            ros::Time ros_time = ros::Time::now();
            controller_manager_->update(ros_time, elapsed_time);
            last_time = current_time;

            // Output
            hardware_interface_->write();

            // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is
            // a rotation vector representation of the tool orientation
            std::vector<double> tool_vector_actual =
                robot_.rt_interface_->robot_state_->getToolVectorActual();

            // Compute rotation angle
            double rx = tool_vector_actual[3];
            double ry = tool_vector_actual[4];
            double rz = tool_vector_actual[5];
            double angle = std::sqrt(std::pow(rx, 2) + std::pow(ry, 2) + std::pow(rz, 2));

            // Broadcast transform
            if (tf_pub.trylock())
            {
                tf_pub.msg_.transforms[0].header.stamp = ros_time;
                if (angle < 1e-16)
                {
                    tf_pub.msg_.transforms[0].transform.rotation.x = 0;
                    tf_pub.msg_.transforms[0].transform.rotation.y = 0;
                    tf_pub.msg_.transforms[0].transform.rotation.z = 0;
                    tf_pub.msg_.transforms[0].transform.rotation.w = 1;
                }
                else
                {
                    tf_pub.msg_.transforms[0].transform.rotation.x = (rx / angle) * std::sin(angle * 0.5);
                    tf_pub.msg_.transforms[0].transform.rotation.y = (ry / angle) * std::sin(angle * 0.5);
                    tf_pub.msg_.transforms[0].transform.rotation.z = (rz / angle) * std::sin(angle * 0.5);
                    tf_pub.msg_.transforms[0].transform.rotation.w = std::cos(angle * 0.5);
                }
                tf_pub.msg_.transforms[0].transform.translation.x = tool_vector_actual[0];
                tf_pub.msg_.transforms[0].transform.translation.y = tool_vector_actual[1];
                tf_pub.msg_.transforms[0].transform.translation.z = tool_vector_actual[2];

                tf_pub.unlockAndPublish();
            }

            // Publish tool velocity
            std::vector<double> tcp_speed = robot_.rt_interface_->robot_state_->getTcpSpeedActual();

            if (tool_vel_pub.trylock())
            {
                tool_vel_pub.msg_.header.stamp = ros_time;
                tool_vel_pub.msg_.twist.linear.x = tcp_speed[0];
                tool_vel_pub.msg_.twist.linear.y = tcp_speed[1];
                tool_vel_pub.msg_.twist.linear.z = tcp_speed[2];
                tool_vel_pub.msg_.twist.angular.x = tcp_speed[3];
                tool_vel_pub.msg_.twist.angular.y = tcp_speed[4];
                tool_vel_pub.msg_.twist.angular.z = tcp_speed[5];

                tool_vel_pub.unlockAndPublish();
            }
        }
    }
    */

    /**
     * Publishes Realtime Messages about the robots' state and end effector states.
     */
    void publishRTMsg()
    {
        ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("manipulator/joint_states", 1);
        std::vector<ros::Publisher> wrench_pubs;
        std::vector<ros::Publisher> tool_vel_pubs;
        for (size_t i = 0; i < robots_.size(); i++)
        {
            wrench_pubs.emplace_back(nh_.advertise<geometry_msgs::WrenchStamped>("wrench"  + std::to_string(i), 1));
            tool_vel_pubs.emplace_back(nh_.advertise<geometry_msgs::TwistStamped>("tool_velocity" + std::to_string(i), 1));
        }
        static tf::TransformBroadcaster br;
        while (ros::ok())
        {
            sensor_msgs::JointState joint_msg;
            std::vector<double> all_positions; 
            std::vector<double> all_velocities; 
            std::vector<double> all_efforts;
            std::vector<std::string> all_names; 
            size_t traj_offset = 0;
            for (auto robot : robots_)
            {
                std::vector<std::string> j_names = robot->getJointNames();
                all_names.insert(all_names.end(), j_names.begin(), j_names.end());
                std::mutex msg_lock;  // The values are locked for reading in the class, so just use a dummy mutex
                std::unique_lock<std::mutex> locker(msg_lock);
                while (!robot->rt_interface_->robot_state_->getDataPublished())
                {
                    rt_msg_cond_.wait(locker);
                }
                joint_msg.header.stamp = ros::Time::now();
                std::vector<double> position = robot->rt_interface_->robot_state_->getQActual();
                std::vector<double> velocity = robot->rt_interface_->robot_state_->getQdActual();
                std::vector<double> effort = robot->rt_interface_->robot_state_->getIActual();
                all_positions.insert(all_positions.end(), position.begin(), position.end());
                for (unsigned int i = traj_offset; i < traj_offset + joint_msg.position.size(); i++)
                {
                    all_positions[i] += joint_offsets_[i];
                }
                all_velocities.insert(all_velocities.end(), velocity.begin(), velocity.end());
                all_efforts.insert(all_efforts.end(), effort.begin(), effort.end());
                traj_offset += joint_msg.position.size();
            }
            joint_msg.position = all_positions;
            joint_msg.velocity = all_velocities;
            joint_msg.effort = all_efforts;
            joint_msg.name = all_names;
            joint_pub.publish(joint_msg);

            for (size_t i = 0; i < robots_.size(); i++)
            {
                auto robot = *robots_[i];
                std::vector<double> tcp_force = robot.rt_interface_->robot_state_->getTcpForce();
                geometry_msgs::WrenchStamped wrench_msg;
                wrench_msg.header.stamp = joint_msg.header.stamp;
                wrench_msg.wrench.force.x = tcp_force[0];
                wrench_msg.wrench.force.y = tcp_force[1];
                wrench_msg.wrench.force.z = tcp_force[2];
                wrench_msg.wrench.torque.x = tcp_force[3];
                wrench_msg.wrench.torque.y = tcp_force[4];
                wrench_msg.wrench.torque.z = tcp_force[5];
                wrench_pubs[i].publish(wrench_msg);

                // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is
                // a rotation vector representation of the tool orientation
                std::vector<double> tool_vector_actual =
                    robot.rt_interface_->robot_state_->getToolVectorActual();

                // Create quaternion
                tf::Quaternion quat;
                double rx = tool_vector_actual[3];
                double ry = tool_vector_actual[4];
                double rz = tool_vector_actual[5];
                double angle = std::sqrt(std::pow(rx, 2) + std::pow(ry, 2) + std::pow(rz, 2));
                if (angle < 1e-16)
                {
                    quat.setValue(0, 0, 0, 1);
                }
                else
                {
                    quat.setRotation(tf::Vector3(rx / angle, ry / angle, rz / angle), angle);
                }

                // Create and broadcast transform
                tf::Transform transform;
                transform.setOrigin(
                    tf::Vector3(tool_vector_actual[0], tool_vector_actual[1], tool_vector_actual[2]));
                transform.setRotation(quat);
                br.sendTransform(
                    tf::StampedTransform(transform, joint_msg.header.stamp, base_frames_[i], tool_frames_[i]));

                // Publish tool velocity
                std::vector<double> tcp_speed = robot.rt_interface_->robot_state_->getTcpSpeedActual();
                geometry_msgs::TwistStamped tool_twist;
                tool_twist.header.frame_id = base_frames_[i];
                tool_twist.header.stamp = joint_msg.header.stamp;
                tool_twist.twist.linear.x = tcp_speed[0];
                tool_twist.twist.linear.y = tcp_speed[1];
                tool_twist.twist.linear.z = tcp_speed[2];
                tool_twist.twist.angular.x = tcp_speed[3];
                tool_twist.twist.angular.y = tcp_speed[4];
                tool_twist.twist.angular.z = tcp_speed[5];
                tool_vel_pubs[i].publish(tool_twist);

                robot.rt_interface_->robot_state_->setDataPublished();
            }
        }
    }

    /**
     * Publishes "Masterboard" data on ROS.
     */
    void publishMbMsg()
    {
        bool warned = false;
        std::vector<ros::Publisher> io_pubs;
        for (size_t i = 0; i < robots_.size(); i++)
        {
            io_pubs.emplace_back(nh_.advertise<ur_msgs::IOStates>("ur_driver/io_states" + std::to_string(i), 1));
        }

        while (ros::ok())
        {
            for (size_t i = 0; i < robots_.size(); i++)
            {
                auto robot = robots_[i];
                ur_msgs::IOStates io_msg;
                std::mutex msg_lock;  // The values are locked for reading in the class, so just use a dummy mutex
                std::unique_lock<std::mutex> locker(msg_lock);
                while (!robot->sec_interface_->robot_state_->getNewDataAvailable())
                {
                    msg_cond_.wait(locker);
                }
                int i_max = 10;
                if (robot->sec_interface_->robot_state_->getVersion() > 3.0)
                    i_max = 18;  // From version 3.0, there are up to 18 inputs and outputs
                for (unsigned int i = 0; i < i_max; i++)
                {
                    ur_msgs::Digital digi;
                    digi.pin = i;
                    digi.state = ((robot->sec_interface_->robot_state_->getDigitalInputBits() & (1 << i)) >> i);
                    io_msg.digital_in_states.push_back(digi);
                    digi.state = ((robot->sec_interface_->robot_state_->getDigitalOutputBits() & (1 << i)) >> i);
                    io_msg.digital_out_states.push_back(digi);
                }
                ur_msgs::Analog ana;
                ana.pin = 0;
                ana.state = robot->sec_interface_->robot_state_->getAnalogInput0();
                io_msg.analog_in_states.push_back(ana);
                ana.pin = 1;
                ana.state = robot->sec_interface_->robot_state_->getAnalogInput1();
                io_msg.analog_in_states.push_back(ana);

                ana.pin = 0;
                ana.state = robot->sec_interface_->robot_state_->getAnalogOutput0();
                io_msg.analog_out_states.push_back(ana);
                ana.pin = 1;
                ana.state = robot->sec_interface_->robot_state_->getAnalogOutput1();
                io_msg.analog_out_states.push_back(ana);
                io_pubs[i].publish(io_msg);

                if (robot->sec_interface_->robot_state_->isEmergencyStopped() or
                    robot->sec_interface_->robot_state_->isProtectiveStopped())
                {
                    if (robot->sec_interface_->robot_state_->isEmergencyStopped() and !warned)
                    {
                        print_error("Emergency stop pressed!");
                    }
                    else if (robot->sec_interface_->robot_state_->isProtectiveStopped() and !warned)
                    {
                        print_error("Robot is protective stopped!");
                    }

                    if (any_executing())
                    {
                        print_error("Aborting trajectory");
                        stop_all_robots();
                        result_.error_code = result_.SUCCESSFUL;
                        result_.error_string = "Robots were halted";
                        goal_handle_.setAborted(result_, result_.error_string);
                    }
                    warned = true;
                }
                else
                    warned = false;

                robot->sec_interface_->robot_state_->finishedReading();
            }
        }
    }
};

int main(int argc, char **argv)
{
    bool use_sim_time = false;
    std::vector<std::string> hosts;
    std::vector<std::string> prefixes;
    int reverse_port = 50001;

    ros::init(argc, argv, "ur_driver");
    ros::NodeHandle nh;
    if (ros::param::get("use_sim_time", use_sim_time))
    {
        print_warning("use_sim_time is set!!");
    }
    // TODO(brycew): change this in the launch file
    if (!(ros::param::get("~robot_ip_addresses", hosts)))
    {
        print_fatal("Could not get robot ip. Please supply it as command line parameter or on the "
                        "parameter server as robot_ip");
        exit(1);
    }
    if (!(ros::param::get("~prefixes", prefixes)))
    {
        print_fatal("Could not get joint prefixes for each robot. Each robot needs a different prefix.");
        exit(1);
    }
    if (prefixes.size() == 0 && hosts.size() > 1)
    {
        print_fatal("If using more than 1 robot, we need to have prefixes for each.");
        exit(1);
    }
    if ((ros::param::get("~reverse_port", reverse_port)))
    {
        if ((reverse_port <= 0) or (reverse_port >= 65535))
        {
            print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default "
                          "value of 50001");
            reverse_port = 50001;
        }
    }
    else
        reverse_port = 50001;

    RosWrapperN interface(hosts, reverse_port, prefixes);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();

    interface.halt();

    exit(0);
}
