#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

from __future__ import print_function
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from std_msgs.msg import Bool
import rospy
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import actionlib
from time import sleep
from tkinter import N
import roslib
roslib.load_manifest('robotiq_2f_gripper_control')


class Robotiq2fClient:
    def __init__(self, gripper_name="gripper") -> None:
        self.gripper_name = gripper_name

        self.ac = actionlib.SimpleActionClient(gripper_name, GripperCommandAction)
        rospy.loginfo("Waiting for action server to start...")
        self.ac.wait_for_server()
        rospy.loginfo("Action server started.")
        self.goal_pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        self.init_reset()
        
        self.gripper_cmd_sub = rospy.Subscriber('/set_gripper_open', Bool, self.gripper_cmd_cb)
        

    def gripper_cmd_cb(self, msg: Bool):
        if msg.data == True:
            self.open()
        else:
            self.close()

    def init_reset(self):
        self.reset()
        rospy.sleep(1)
        self.reset()
        rospy.sleep(1)
        self.activate()
        rospy.loginfo('init finished.')

    def reset(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 0x0
        self.goal_pub.publish(command)

    def activate(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 0x1
        self.goal_pub.publish(command)

    def close(self, pos=-0.01, effort=60):
        goal = GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.ac.send_goal(goal)

    def open(self, pos=0.085, effort=60):
        goal = GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.ac.send_goal(goal)

    def wait_result_and_get_state(self):
        finished_before_timeout = self.ac.wait_for_result(rospy.Duration(30))

        if finished_before_timeout:
            state = self.ac.get_state()
            rospy.loginfo(f"Action finished: {state}")
            return state

        return None


def simple_gripper_test():

    gripper_name = "gripper"

    rospy.init_node('test_robotiq_2f_gripper_action_server')

    ac = actionlib.SimpleActionClient(gripper_name, GripperCommandAction)
    rospy.loginfo("Waiting for action server to start...")
    ac.wait_for_server()
    rospy.loginfo("Action server started.")

    rospy.loginfo("Action - Close ...")
    goal = GripperCommandGoal()
    goal.command.position = -0.01
    goal.command.max_effort = 100.0
    ac.send_goal(goal)

    # wait for the action to return
    finished_before_timeout = ac.wait_for_result(rospy.Duration(30))

    if finished_before_timeout:
        state = ac.get_state()
        rospy.loginfo(f"Action finished: {state}")
    else:
        rospy.loginfo("Action did not finish before the time out.")

    rospy.sleep(3)

    rospy.loginfo("Action - Open ...")
    as_goal = GripperCommandGoal()
    as_goal.command.position = 0.085
    as_goal.command.max_effort = 100.0
    ac.send_goal(as_goal)

    # wait for the action to return
    finished_before_timeout = ac.wait_for_result(rospy.Duration(30))

    if finished_before_timeout:
        state = ac.get_state()
        rospy.loginfo(f"Action finished: {state}")
    else:
        rospy.loginfo("Action did not finish before the time out.")


def class_gripper_test():
    rospy.init_node('test_robotiq_2f_gripper_action_server')
    robotiq = Robotiq2fClient()
    rospy.sleep(3)
    robotiq.init_reset()
    rospy.sleep(3)
    robotiq.close()
    rospy.sleep(3)
    robotiq.open()
    rospy.sleep(3)


def main():
    rospy.init_node('robotiq_client_node')
    robotiq = Robotiq2fClient()
    rospy.spin()


if __name__ == '__main__':
    # simple_gripper_test()
    # class_gripper_test()
    main()
