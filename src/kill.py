import subprocess
import time

def kill():
    """Forcefully terminate all ROS and Gazebo processes."""
    print("\nTerminating all processes and cleaning up ROS services...")

    subprocess.run(["pkill", "-f", "ros"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gazebo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "python3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  # Kills any lingering Python ROS nodes
    subprocess.run(["pkill", "-f", "pick_place_simple_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(2)

    subprocess.run(["killall", "-9", "rosmaster", "roslaunch", "gzserver", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("All ROS services and processes terminated.")
    return 0

if __name__ == "__main__":
    kill()