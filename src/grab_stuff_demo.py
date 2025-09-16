#!/usr/bin/env python

# Author: Ben Davis
# Create: 8/29/25
# Description: repeated grasp with recordings of grasp force and digit image to collect training data


# imports
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False

from calendar import month_abbr
import os, sys
import string
from helperFunction.utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix, normalize, hat


from datetime import datetime
import numpy as np
import time
import scipy
import pickle

from netft_utils.srv import *
from suction_cup.srv import *
from edg_ur10_robotiq.srv import GetImage
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool
import geometry_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
import cv2
import config

# actionlib
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


current_dir = os.path.dirname(os.path.abspath(__file__))
helper_path = os.path.join(current_dir, "helperFunction")
sys.path.append(helper_path)

from FT_callback_helper import FT_CallbackHelp
from fileSaveHelper import fileSaveHelp
from rtde_helper import rtdeHelp
from adaptiveMotion import adaptMotionHelp

def main(args):

  SYNC_RESET = 0
  SYNC_START = 1
  SYNC_STOP = 2
  
  deg2rad = np.pi / 180.0

  np.set_printoptions(precision=4)

  # controller node
  rospy.init_node('edg_experiment')

  # gripper
  print('initializing gripper...')
  action_name = 'command_robotiq_action'
  # robotiq_driver = Robotiq2FingerGripperDriver(comport=args.comport, client_name=action_name)
  # robotiq_driver.wait_for_server()
  robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
  robotiq_client.wait_for_server()
  print('gripper initialized')

  # Setup helper functions
  FT_help = FT_CallbackHelp() # it deals with subscription.
  rospy.sleep(0.5)
  file_help = fileSaveHelp()
  rospy.sleep(0.5)
  rtde_help = rtdeHelp(125)
  adpt_help = adaptMotionHelp(d_w = config.TWIST_SPEED,d_lat = 10e-3, d_z = 6e-5) # need to change d_z to change the speed of the robot
  rospy.sleep(0.5)
  rtde_help.setTCPoffset(config.GRIPPER_OFFSET)

  # Set the synchronization Publisher
  syncPub = rospy.Publisher('sync', Int8, queue_size=1)

  # grasp force publisher
  graspForcePub = rospy.Publisher('grasp_force', Float32, queue_size=1)
  isGraspingPub = rospy.Publisher('is_grasping', Bool, queue_size=1)

  print("Wait for the data_logger to be enabled")
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False) # reset Data Logger just in case
  print("Wait for digit frame toggle service")
  rospy.wait_for_service('capture_digit_frame')
  capture_digit = rospy.ServiceProxy('capture_digit_frame', SetBool)
  rospy.wait_for_service('toggle_digit_frame')
  toggle_digit = rospy.ServiceProxy('toggle_digit_frame', SetBool)
  rospy.wait_for_service('get_last_image')
  get_last_image = rospy.ServiceProxy('get_last_image', GetImage)
  rospy.sleep(1)
  file_help.clearTmpFolder()        # clear the temporary folder
  datadir = file_help.ResultSavingDirectory


  # Set the pose A
  
  positionA = config.POSITION_D
  orientationA = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/4,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA, orientationA)
  # try block so that we can have a keyboard exception
  try:

    input("Press <Enter> to go start pose")
    # open
    goal = CommandRobotiqGripperGoal()
    goal.position = 0.1
    goal.speed = 0
    goal.force = 0
    robotiq_client.send_goal(goal)
    robotiq_client.wait_for_result()
    rtde_help.goToPose(poseA)

    input("Press <Enter> to go start expeirment")
    # set biases now
    try:
      FT_help.setNowAsBias()
      rospy.sleep(0.1)
    except:
      print("set now as offset failed, but it's okay")

    # start data logging
    dataLoggerEnable(True)
    syncPub.publish(SYNC_START)
    rospy.sleep(0.2)
    # record_data(capture_digit, graspForcePub, isGraspingPub, ppc=config.POINTS_PER_CAPTURE, grasp_force=-1, is_grasping=False)

    # close gripper
    input("Press <Enter> to close gripper")
    rospy.sleep(3)
    # close gripper
    goal = CommandRobotiqGripperGoal()
    goal.position = 0.005
    goal.speed = 50
    goal.force = 150
    robotiq_client.send_goal(goal)
    robotiq_client.wait_for_result()


    # record one frame video data
    rospy.sleep(2)

    record_data(capture_digit, graspForcePub, isGraspingPub, ppc=config.POINTS_PER_CAPTURE, grasp_force=-1, is_grasping=False)

    rospy.sleep(0.5)

    input("Press <Enter> to open gripper")

    # open
    goal = CommandRobotiqGripperGoal()
    goal.position = 0.07
    goal.speed = 100
    goal.force = 0
    robotiq_client.send_goal(goal)
    robotiq_client.wait_for_result()
    rospy.sleep(0.2)

    

    # stop data logging
    rtde_help.goToPose(poseA)
    syncPub.publish(SYNC_STOP)
    dataLoggerEnable(False)
    rospy.sleep(1)

    # save data and clear the temporary folder
    # file_help.saveDataParams(args, appendTxt='Simple_experiment_'+'depth_'+str(args.depth)+'_cycle_'+str(args.cycle))
    file_help.clearTmpFolder()

    print("============ Python UR_Interface demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  
  
# function for saving certain number of frames
def save_frames(capture_digit, wait_time=0.001):
  capture_digit(True)
  rospy.sleep(wait_time)  # Wait for the specified save period

def record_data(capture_digit, graspForcePub, isGraspingPub, grasp_force, is_grasping, ppc=config.POINTS_PER_CAPTURE):
  for i in range(ppc):
    save_frames(capture_digit)
    graspForcePub.publish(grasp_force)
    isGraspingPub.publish(is_grasping)


if __name__ == '__main__':  
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--depth', type=int, help='argument for test type', default= 0)
  parser.add_argument('--author', type=str, help='argument for str type', default= "EDG")
  parser.add_argument('--cycle', type=int, help='the number of cycle to apply', default = 1)
  parser.add_argument('--comport', type=str, help='gripper comport', default='/dev/ttyUSB0')

  args = parser.parse_args()    
  main(args)