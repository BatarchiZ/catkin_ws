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

def launch_tracker():
    print("[DEBUG] Launching new tracker node...")
    subprocess.Popen(["/usr/bin/python3", "_class_object_tracker_copy.py"])
    time.sleep(3)  # Wait for the node to start

def get_object_position():
    rospy.wait_for_service("/get_object_position")
    try:
        get_position = rospy.ServiceProxy("/get_object_position", Trigger)
        response = get_position()
        if response.success:
            print(f"[INFO] Object Position: {response.message}")
            x1, y1, z1 = map(float, response.message.split(', '))
            return x1, y1, z1
        else:
            print(f"[WARN] Failed to get object position: {response.message}")
            return None, None, None
    except rospy.ServiceException as e:
        print(f"[ERROR] Service call failed: {e}")
        return None, None, None

def launch_euclid_distance():
    print("[DEBUG] Launching new distance_calculator node...")
    subprocess.Popen(["/usr/bin/python3", "_class_euclidean_distance_calculator.py"])
    time.sleep(3)  # Wait for the node to start


def get_euclidean_distance():
    rospy.wait_for_service("/get_gripper_distance")
    try:
        get_distance = rospy.ServiceProxy("/get_gripper_distance", Trigger)
        response = get_distance()
        if response.success:
            print(f"[INFO] Distance between cobot and object: {response.message}")
            # distnace = map(float, response.message.split(', '))
            return float(response.message)
        else:
            print(f"[WARN] Failed to get distance: {response.message}")
            return None
    except rospy.ServiceException as e:
        print(f"[ERROR] Service call failed: {e}")
        return None


def check_grasp_success(distance):
    # distance = get_euclidean_distance()
    if distance <= (0.055**2 + 0.055**2 + 0**2)**(0.5):
        print("OBJECT WITHIN THE GRIPPER")
        return True
    else:
        return False

    # threshold = 0.02
    # rospy.loginfo("Checking grasp success...")

    # x1, y1, z1 = get_object_position()
    # if x1 is None:
    #     return False
    
    # move_cobot(x, y, z + 0.1)

    # x2, y2, z2 = get_object_position()
    # if x2 is None:
    #     return False
    

    # displacement = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) 
    # rospy.loginfo(f"Object displacement: {displacement}")

    # if displacement > threshold:
    #     print("OBJECT_MOVED", displacement)
    # else:
    #     print("no_move"), displacement
    
    # return displacement > threshold

class GraspEnv(gym.Env):
    def __init__(self, image_path):
        print("<INFO> CONSTRUCTOR")
        super(GraspEnv, self).__init__()
        self.image_path = image_path  # Path to the real-time image
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)  # Image as input
        self.action_space = spaces.Box(low=np.array([0, 0.05]), high=np.array([1, 0.7]), dtype=np.float32)  # Normalized (x, y)
        print("<INFO> CONSTRUCTOR END")
        self.min_distance = 2

    def reset(self):
        print("<INFO> RESETTING")
        kill_all_ros_processes()  # Ensure a clean restart
        start_world()  # Relaunch the simulation

        launch_tracker() # Position node for checking object position
        launch_euclid_distance()

        self.obj_x1, self.obj_y1, self.obj_z1 = get_object_position()
        time.sleep(2)  # Allow some time for startup
        
        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Image file not found: {self.image_path}")
        
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            raise ValueError("Failed to load image from path.")
        print("<INFO> RESETTING END")
        return self.image  # Return as observation
    
    def _my_reset(self):
        self.image = self.reset()
        return self.image

    def step(self, action):
        print("<INFO> STEP")
        reward = 0
        z = 0.5
        x, y = action  # RL chooses (x, y)
        
        fail = move_cobot(x, y, z)
        if fail == -1: 
            self._my_reset()
            reward = -1
            return self.image, reward, True, {}
        
        obj_x2, obj_y2, obj_z2 = get_object_position()
        displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 + (obj_y2 - self.obj_y1) ** 2 + (obj_z2 - self.obj_z1) ** 2)
        threshold = 0.055
        if displacement > threshold:
            print("<<<<FAIL>>>> COBOT MOVED OBJECT")
            self._my_reset()
            reward = -10
            return self.image, reward, False, {}
        
        # self.image = cv2.imread(self.image_path)
        
        distance = get_euclidean_distance()
        success = check_grasp_success(distance)
        if success == 1:
            reward = 100
            return self.image, reward, True, {}
        elif success == 0:
            reward = max((self.min_distance - distance), -0.2)
            self.min_distance = min(self.min_distance, distance)

        
        print("<INFO> STEP END")
        return self.image, reward, False, {}


# import time

# if __name__ == "__main__":
#     try: 
#         env = GraspEnv(image_path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")
#         env.reset()
#         total_start_time = time.time()  # Start timing the total execution

#         for i in range(3000):
#             step_start_time = time.time()  # Start timing this step
            
#             # obs = env.reset()
#             # print("Successfully reset world")
#             action = env.action_space.sample()
#             print(action)
#             obs, reward, done, _ = env.step(action)

#             step_time = time.time() - step_start_time  # Calculate step time
#             print(f"Iteration {i+1}: Action={action}, Reward={reward}, Step Time={step_time:.4f} sec")

#         total_time = time.time() - total_start_time  # Calculate total execution time
#         print(f"\nTotal execution time for 3000 iterations: {total_time:.2f} sec")
#         print(f"Average time per step: {total_time / 3000:.4f} sec")
        
#         kill()
#     except KeyboardInterrupt:
#         total_time = time.time() - total_start_time  # Calculate total execution time
#         print(f"\nTotal execution time for x iterations: {total_time:.2f} sec")
#         print(f"Average time per step: {total_time / i:.4f} sec")
#         kill()

        
