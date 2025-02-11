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
        model.learn(total_timesteps=100, callback=callback)

        # Save model after training
        model.save("z_models/a2c_testing_110225.zip")

        env.close()
        kill()
    except KeyboardInterrupt:
        model.save("z_models/a2c_testing_110225.zip")
        kill()
    