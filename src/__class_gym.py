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

def check_grasp_success(x, y, z):
    threshold = 0.02
    rospy.loginfo("Checking grasp success...")

    x1, y1, z1 = get_object_position()
    if x1 is None:
        return False
    
    move_cobot(x, y, z + 0.1)
    
    # Uncomment to check whether item is moved
    # move_cobot(1.0, 0.0, 0.5)
    time.sleep(1)

    x2, y2, z2 = get_object_position()
    if x2 is None:
        return False
    

    displacement = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) 
    rospy.loginfo(f"Object displacement: {displacement}")

    if displacement > threshold:
        print("OBJECT_MOVED", displacement)
    else:
        print("no_move"), displacement
    
    return displacement > threshold

class GraspEnv(gym.Env):
    def __init__(self, image_path):
        print("<INFO> CONSTRUCTOR")
        super(GraspEnv, self).__init__()
        self.image_path = image_path  # Path to the real-time image
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)  # Image as input
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)  # Normalized (x, y)
        print("<INFO> CONSTRUCTOR END")

    def reset(self):
        print("<INFO> RESETTING")
        kill_all_ros_processes()  # Ensure a clean restart
        start_world()  # Relaunch the simulation

        launch_tracker() # Position node for checking object position

        time.sleep(5)  # Allow some time for startup
        
        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Image file not found: {self.image_path}")
        
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            raise ValueError("Failed to load image from path.")
        print("<INFO> RESETTING END")
        return self.image  # Return as observation
    
    def _my_reset(self):
        print("<INFO> RESETTING")
        kill_all_ros_processes()  # Ensure a clean restart
        start_world()  # Relaunch the simulation

        launch_tracker() # Position node for checking object position

        time.sleep(5)  # Allow some time for startup
        
        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Image file not found: {self.image_path}")
        
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            raise ValueError("Failed to load image from path.")
        print("<INFO> RESETTING END")
        return self.image  # Return as observation

    def step(self, action):
        reward = 0

        print("<INFO> STEP")
        z = 0.5
        x, y = action  # RL chooses (x, y)

        x1, y1, z1 = get_object_position()
        fail = move_cobot(x, y, z)
        if fail == -1: 
            # self._my_reset()
            return self.image, 0, True, {}
        x2, y2, z2 = get_object_position()
        displacement = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        threshold = 0.02


        if displacement > threshold:
            print("COBOT MOVED OBJECT")
            # self._my_reset()
            reward = -100
            return self.image, reward, False, {}
        

        success = check_grasp_success(x, y, z)

        self.image = cv2.imread(self.image_path)
        if self.image is None:
            raise ValueError("Failed to load image from path.")
        
        if success == 1:
            reward = 100
            return self.image, reward, True, {}

        elif success == 0:
            distance = -1
            reward = 1 / distance
        
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

        
