#!/usr/bin/python3

import gym
import numpy as np
import cv2
from gym import spaces
import time
import os
from gazebo_msgs.msg import ModelStates

from kill import kill

import rospy

from main_copy import kill_all_ros_processes, start_world, move_cobot


import subprocess
import rospy
import time
from std_srvs.srv import Trigger

def launch_gripper_tracker():
    print("[DEBUG] Launching new gripper_tracker node...")
    subprocess.Popen(["/usr/bin/python3", "_class_gripper_right_position.py"])
    time.sleep(3)  # Wait for the node to start

def get_gripper_disposition():
    rospy.wait_for_service("/get_gripper_right_error")
    try:
        error = rospy.ServiceProxy("/get_gripper_right_error", Trigger)
        response = error()

        if response.success:
            print(f"[INFO] Gripper Error: {response.message}")
            err = float(response.message)
            return err
        else:
            print(f"[WARN] Failed to get object position: {response.message}")
            return None
    except rospy.ServiceException as e:
        print(f"[ERROR] Service call failed: {e}")
        return None
    
launch_gripper_tracker()
print(get_gripper_disposition())