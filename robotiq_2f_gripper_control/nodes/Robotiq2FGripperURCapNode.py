#!/usr/bin/env python3
import os
import sys
import socket
import threading
import rospy

from robotiq_2f_gripper_control.baseRobotiq2FGripper import robotiqbaseRobotiq2FGripper
from robotiq_2f_gripper_control.robotiq_2f_gripper_urcap_bridge import Robotiq2FGripperURCapBridge
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool

class Robotiq2FGripperURapNode(robotiqbaseRobotiq2FGripper):
  def __init__(self, ur_address='10.2.0.50') -> None:
    self.gripper = Robotiq2FGripperURCapBridge(ur_address)
    self.gripper.activate()
    self.lock=threading.RLock()
    self.cmd = None
  
  def getStatus(self):
    gripper:Robotiq2FGripperURCapBridge = self.gripper
    msg = Robotiq2FGripper_robot_input()
    msg.gACT = int(gripper._get_var(gripper.ACT))
    msg.gGTO = int(gripper._get_var(gripper.GTO))
    msg.gSTA = int(gripper._get_var(gripper.STA))
    msg.gOBJ = int(gripper._get_var(gripper.OBJ))
    msg.gFLT = int(gripper._get_var(gripper.FLT))
    msg.gPR  = int(gripper._get_var(gripper.PRE))
    msg.gPO  = int(gripper._get_var(gripper.POS))
    msg.gCU  = int(gripper._get_var(gripper.FOR))
    return msg
  
  def refreshCommand(self, command:Robotiq2FGripper_robot_output):
    """Function to update the command which will be sent during the next sendCommand() call."""

    #Limit the value of each variable
    command = self.verifyCommand(command)
    cmd= {'ACT':command.rACT,
          'GTO':command.rGTO,
          'ATR':command.rATR,
          'PRE':command.rPR,
          'SPE':command.rSP,
          'FOR':command.rFR,
          }
    with self.lock:
      self.cmd = cmd
  
  def sendCommand(self):
    """Send the command to the Gripper."""
    with self.lock:
      if self.cmd is not None:
        self.gripper._set_vars(self.cmd)
        self.cmd = None

  def gripper_cmd_cb(self, msg: Bool):
    if msg.data == True:
        self.gripper.open()
    else:
        self.gripper.close()

def mainLoop(ur_address):
  # Gripper is a C-Model that is connected to a UR controller with the Robotiq URCap installed. 
  # Commands are published to port 63352 as ASCII strings.
  gripper = Robotiq2FGripperURapNode(ur_address)
  
  # The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
  pub = rospy.Publisher('Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, queue_size=1)

  # The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
  sub = rospy.Subscriber('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, gripper.refreshCommand)

  sub_bool = rospy.Subscriber('set_gripper_open', Bool, gripper.gripper_cmd_cb)
  

  #We loop
  while not rospy.is_shutdown():

    #Get and publish the Gripper status
    status = gripper.getStatus()
    if status is not None:
      pub.publish(status)

    #Wait a little
    rospy.sleep(0.002)

    #Send the most recent command
    gripper.sendCommand()

    #Wait a little
    rospy.sleep(0.002)


if __name__ == '__main__':
  rospy.init_node('cmodel_urcap_driver')
  try:
    mainLoop(sys.argv[1])
  except rospy.ROSInterruptException: pass