from gym_class import GraspEnv
from kill import kill



import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from gym_class import GraspEnv  # Ensure your GraspEnv is properly implemented

if __name__ == "__main__":
    try: 
        # Initialize environment
        env = GraspEnv(image_path="/home/is/catkin_ws/src/z_output/recent_frame.jpg")
        # env = DummyVecEnv([lambda: env])  # Wrap in DummyVecEnv for vectorized training

        # Define PPO model
        # model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_grasp_tensorboard/")
        # model = PPO("MlpPolicy", env, verbose=1)


        # Train the agent
        TIMESTEPS = 10  # Adjust based on your needs
        model.learn(total_timesteps=TIMESTEPS)

        # Save the model
        model.save("ppo_grasp")

        # Close the environment
        env.close()
    except KeyboardInterrupt:
        kill()


    