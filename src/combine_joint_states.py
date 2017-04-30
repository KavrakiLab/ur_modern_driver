#!/usr/bin/env python

import sys
import copy
import rospy
from sensor_msgs.msg import JointState

class CombineJointStates(object):


    def callback_left_arm(self, data):
        rospy.logdebug('left joint states = {}'.format(data))
        self.left_arm_data = data
    def callback_right_arm(self, data):
        rospy.logdebug('right joint states = {}'.format(data))
        self.right_arm_data = data
    def callback_left_gripper(self, data):
        rospy.logdebug('left joint states = {}'.format(data))
        self.left_gripper_data = data
    def callback_right_gripper(self, data):
        rospy.logdebug('right joint states = {}'.format(data))
        self.right_gripper_data = data
    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.Subscriber("left_manipulator/joint_states", JointState, self.callback_left_arm)
        rospy.Subscriber("right_manipulator/joint_states", JointState , self.callback_right_arm)
        rospy.Subscriber("left_gripper/joint_states", JointState, self.callback_left_gripper)
        rospy.Subscriber("right_gripper/joint_states", JointState , self.callback_right_gripper)

        # spin() simply keeps python from exiting until this node is stopped
        joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            output = JointState()
            output.name=list()
            output.position=list()
            output.velocity=list()
            output.effort=list()
            if self.left_arm_data is not None:
                output.header = self.left_arm_data.header
                output.name = output.name + self.left_arm_data.name
                output.position = output.position + list(self.left_arm_data.position)
                output.velocity = output.velocity + list(self.left_arm_data.velocity)
                output.effort = output.velocity + list(self.left_arm_data.effort)
            if self.right_arm_data is not None:
                output.header = self.right_arm_data.header
                output.name = output.name + self.right_arm_data.name
                rospy.logdebug('output.name = {}'.format(output.name))
                output.position = output.position + list(self.right_arm_data.position)
                output.velocity = output.velocity + list(self.right_arm_data.velocity)
                output.effort = output.velocity + list(self.right_arm_data.effort)
            if self.left_gripper_data is not None:
                output.header = self.left_gripper_data.header
                output.name = output.name + self.left_gripper_data.name
                output.position = output.position + list(self.left_gripper_data.position)
                output.velocity = output.velocity + list(self.left_gripper_data.velocity)
                output.effort = output.velocity + list(self.left_gripper_data.effort)
            if self.right_gripper_data is not None:
                output.header = self.right_gripper_data.header
                output.name = output.name + self.right_gripper_data.name
                output.position = output.position + list(self.right_gripper_data.position)
                output.velocity = output.velocity + list(self.right_gripper_data.velocity)
                output.effort = output.velocity + list(self.right_gripper_data.effort)

            joint_state_pub.publish(output)
            rospy.logdebug('output = {}'.format(output))

            rate.sleep()

    def __init__(self):
        self.left_arm_data = None
        self.right_arm_data = None
        self.left_gripper_data = None
        self.right_gripper_data = None

if __name__ == '__main__':
      rospy.init_node('joint_states_combine')
      joint_states_combine = CombineJointStates()

      joint_states_combine.listener()
