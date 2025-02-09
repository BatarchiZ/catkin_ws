import csv
import os
from stable_baselines3.common.callbacks import BaseCallback

class SaveMetricsCallback(BaseCallback):
    def __init__(self, log_dir="logs", verbose=1):
        super().__init__(verbose)
        self.log_dir = log_dir
        self.csv_file = os.path.join(log_dir, "/home/is/catkin_ws/src/____logs/training_metrics.csv")
        self.first_write = True  # Ensures headers are written only once

    def _on_step(self) -> bool:
        # Get the episode reward
        reward = self.locals["rewards"]  # List of rewards per step
        loss = self.model.logger.name_to_value.get("loss/value_loss", None)  # PPO/A2C loss

        # Ensure the log directory exists
        os.makedirs(self.log_dir, exist_ok=True)

        # Write data to CSV
        with open(self.csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)

            # Write header if file is empty
            if self.first_write:
                writer.writerow(["Step", "Reward", "Loss"])
                self.first_write = False

            writer.writerow([self.num_timesteps, sum(reward), loss])

        return True  # Continue training
