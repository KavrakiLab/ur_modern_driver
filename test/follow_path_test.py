#!/usr/bin/env python

import sys
import rospy
import actionlib
from rospy_message_converter import message_converter
import rospkg
from trajectory_msgs.msg import JointTrajectory
from yaml import load

import ur_modern_driver.msg

def read_in_path():
    rospack  = rospkg.RosPack()
    f  = open(rospack.get_path("ur_modern_driver") + "/test/test_traj.yaml")
    trajectory = load(f)
    f.close()
    path = message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectory', trajectory)
    return path

def follow_path(path):
    client = actionlib.SimpleActionClient('realtime_follower', ur_modern_driver.msg.FollowWaypointsAction)
    client.wait_for_server()
    goal = ur_modern_driver.msg.FollowWaypointsGoal(waypoints = path)

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('follow_path_test_client')
        result = follow_path(read_in_path())
        print("Result:", result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
