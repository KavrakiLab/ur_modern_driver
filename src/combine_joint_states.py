#!/usr/bin/env python

import sys
import copy
import rospy
from sensor_msgs.msg import JointState

class CombineJointStates(object):


    def callback_left(self, data):
        rospy.logdebug('left joint states = {}'.format(data))
        self.left_data = data
    def callback_right(self, data):
        rospy.logdebug('right joint states = {}'.format(data))
        self.right_data = data
    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.Subscriber("left_manipulator/joint_states", JointState, self.callback_left)
        rospy.Subscriber("right_manipulator/joint_states", JointState , self.callback_right)

        # spin() simply keeps python from exiting until this node is stopped
        joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.left_data is not None and self.right_data is not None:
                output = JointState()
                output.header = self.left_data.header
                output.name = self.left_data.name + self.right_data.name
                output.position = self.left_data.position + self.right_data.position
                output.velocity = self.left_data.velocity + self.right_data.velocity
                output.effort = self.left_data.effort + self.right_data.effort
                rospy.loginfo('output = {}'.format(output))
                joint_state_pub.publish(output)
            rate.sleep()

    def __init__(self):
        self.left_data = None
        self.right_data = None

if __name__ == '__main__':
      rospy.init_node('joint_states_combine')
      joint_states_combine = CombineJointStates()

      joint_states_combine.listener()
