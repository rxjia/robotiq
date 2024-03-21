#!/usr/bin/env python3

from __future__ import print_function
import rospy

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import Bool

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg


class Robotiq2fClient:
    def __init__(self, gripper_name="gripper") -> None:
        self.gripper_name = gripper_name

        self.ac = actionlib.SimpleActionClient(gripper_name, GripperCommandAction)
        rospy.loginfo("Waiting for action server to start...")
        self.ac.wait_for_server()
        rospy.loginfo("Action server started.")
        self.gripper_cmd_sub = rospy.Subscriber('/set_gripper_open', Bool, self.gripper_cmd_cb)
        self.goal_pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        self.reset()
        rospy.sleep(0.3)
        self.reset()
        rospy.sleep(1)
        self.activate()
        rospy.loginfo('init ok')

    def reset(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT=0x0
        self.goal_pub.publish(command)

    def activate(self):
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT=0x1
        self.goal_pub.publish(command)

    def gripper_cmd_cb(self, msg: Bool):
        if msg.data == True:
            self.open()
        else:
            self.close()

    def close(self, pos=-0.01, effort=100):
        goal = GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.ac.send_goal(goal)

    def open(self, pos=0.085, effort=100):
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
    finished_before_timeout = ac.wait_for_result(rospy.Duration(30));

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
    finished_before_timeout = ac.wait_for_result(rospy.Duration(30));

    if finished_before_timeout:
        state = ac.get_state()
        rospy.loginfo(f"Action finished: {state}")
    else:
        rospy.loginfo("Action did not finish before the time out.")


def class_gripper_test():
    rospy.init_node('test_robotiq_2f_gripper_action_server')
    robotiq = Robotiq2fClient()
    robotiq.close()
    rospy.sleep(3)
    robotiq.open()
    rospy.sleep(3)


def main():
    rospy.init_node('robotiq_client_node')
    rospy.sleep(0.3)
    robotiq = Robotiq2fClient()
    rospy.spin()


if __name__ == '__main__':
    # simple_gripper_test()
    # class_gripper_test()
    main()