# from gym_class import GraspEnv
from kill import kill



import gymnasium as gym
from stable_baselines3 import PPO, SAC, A2C
from stable_baselines3.common.vec_env import DummyVecEnv
from __class_gym import GraspEnv  # Ensure your GraspEnv is properly implemented

import pandas as pd
import time

from stable_baselines3 import A2C

from _class_save_training import SaveMetricsCallback

if __name__ == "__main__":
    try:
        env = GraspEnv(image_path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")
        model = A2C("MlpPolicy", env, verbose=1, n_steps=1)

        # Initialize callback
        callback = SaveMetricsCallback(log_dir="/home/is/catkin_ws/src/____logs/")

        # Train for exactly one movement (one step)
        model.learn(total_timesteps=500, callback=callback)

        # Save model after training
        model.save("z_models/a2c_testing_100225.zip")

        env.close()
        kill()
    except KeyboardInterrupt:
        kill()



# if __name__ == "__main__":
#     try:
#         env = GraspEnv(image_path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")
#         model = PPO("MlpPolicy", env, verbose=1)

#         # Define CSV log file
#         log_file = "ppo_training_log.csv"
#         log_data = []

#         TIMESTEPS = 1  # Adjust based on your needs

#         for i in range(1):
#             start_time = time.time()
#             print(f"\nTraining iteration {i}...")
#             model.learn(total_timesteps=TIMESTEPS)

#             # Log training data
#             log_data.append({
#                 "Iteration": i,
#                 "Reward": env.get_reward(),
#                 "Time Taken": time.time() - start_time
#             })

#             # Save logs periodically
#             pd.DataFrame(log_data).to_csv(log_file, index=False)

#             # Save model
#             model.save(f"ppo_grasp_iteration_{i}.zip")
#             print(f"Iteration {i} complete. Model saved.")

#         env.close()
#     except KeyboardInterrupt:
#         kill()



    