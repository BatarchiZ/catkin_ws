#!/usr/bin/python3

import gym
import numpy as np
import cv2
from gym import spaces
import time
import os

from main_copy import kill_all_ros_processes, start_world
from cobot_controls import move_cobot, control_gripper
from service_server_control import *



class GraspEnv(gym.Env):
    def __init__(self, image_path):
        print("<INFO> CONSTRUCTOR")
        super(GraspEnv, self).__init__()
        self.image_path = image_path  # Path to the real-time image
        self.observation_space = spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8)  # Image as input
        self.action_space = spaces.Box(low=np.array([0, 0.05]), high=np.array([1, 0.7]), dtype=np.float32)  # Normalized (x, y)
        print("<INFO> CONSTRUCTOR END")
    def reset(self):
        print("<INFO> RESETTING")
        kill_all_ros_processes()  # Ensure a clean restart
        start_world()  # Relaunch the simulation

        launch_tracker() # Position node for checking object position
        launch_euclid_distance()
        launch_gripper_tracker()

        self.min_distance = 2

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
        
        # COBOT may get jammed -
        # fail is when movement execution was longer that 130 seconds
        fail = move_cobot(x, y, z)
        if fail == -1: 
            # self._my_reset()
            reward = -1
            return self.image, reward, True, {}
        
        # CHECK whether object was moved unintentionally
        # IF moved -> restart environment 
        obj_x2, obj_y2, obj_z2 = get_object_position()
        displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 + (obj_y2 - self.obj_y1) ** 2 + (obj_z2 - self.obj_z1) ** 2)
        threshold = 0.055
        if displacement > threshold:
            print("<<<<FAIL>>>> COBOT MOVED OBJECT")
            # self._my_reset()
            reward = -10
            return self.image, reward, True, {}
        
        # CLOSE gripper
        control_gripper(0.055)
        
        # self.image = cv2.imread(self.image_path)
        distance = get_euclidean_distance()

        # CHECK whether gripper closed full amount
        # IF did not close properly 
        # THEN we grasped object
        success = self.check_grasp_success(x, y, z, distance)
        if success:
            reward = 10
            return self.image, reward, True, {}  # Restarts environment
        else:
            if 0 <= distance <= (0.1**2 + 0.1**2 + 0**2) ** (0.5): # We are very close # 0.1 is length of gripper ## TUT LUSCHE MINIMIZE NOT THECENTER OF THE GRIPEPR BUT THE RIGHTMOST POINT AND THEN DURING GRIP JUST MOVE LEFT
                reward = 0
            else :
                reward = (self.min_distance - distance) # The closer we are the larger the reward. The further we are the bigger the penalty. 

            self.min_distance = min(self.min_distance, max(distance, (0.1**2 + 0.1**2 + 0**2)**(1/2)))

        # OPEN gripper for the next movement
        control_gripper(0.0)
        print("<INFO> STEP END")
        return self.image, reward, False, {}

    def check_grasp_success(self, x, y, z, distance):
        # time.sleep(1)
        error = get_gripper_disposition()
        # print(error)
        # print(error >= 0.0075)
        if (error >= 0.0075): 
            print(error, distance)
        threshold_error = 0.0075
        if error >= threshold_error: # Check 1 - grippers do not connect
            move_cobot(x, y, z + 0.1)
            obj_x2, obj_y2, obj_z2 = get_object_position()
            displacement = np.sqrt((obj_x2 - self.obj_x1) ** 2 + (obj_y2 - self.obj_y1) ** 2 + (obj_z2 - self.obj_z1) ** 2)
            threshold_displacement = 0.055
            if displacement >= threshold_displacement: # Check 2 - object moves with cobot
                return True


        return False


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

        
# rostopic pub -1 /cobot/gripper_controller/command trajectory_msgs/JointTrajectory '
# joint_names: ["gripper_right_joint"]
# points:
# - positions: [0.55]
#   time_from_start: {{secs: {1}, nsecs: 0}}