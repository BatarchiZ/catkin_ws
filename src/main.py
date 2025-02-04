import subprocess
import os
import signal
import time

def kill_all_ros_processes():
    """Forcefully terminate all ROS and Gazebo processes."""
    print("\nTerminating all processes and cleaning up ROS services...")

    # Kill all known ROS-related processes
    subprocess.run(["pkill", "-f", "ros"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gazebo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "python3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  # Kills any lingering Python ROS nodes
    subprocess.run(["pkill", "-f", "pick_place_simple_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Give some time for processes to terminate
    time.sleep(2)

    # Force kill anything that didn't exit
    subprocess.run(["killall", "-9", "rosmaster", "roslaunch", "gzserver", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("All ROS services and processes terminated.")
    return 0

def start_world():
    print("Launching robot simulation...")
    sim_process = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch cobot_sim spawn_robot_2Dcamera_traj_controller_world.launch"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(sim_process)

    time.sleep(5)  # Wait for Gazebo to start
    
    return 0

processes = []  # List to store subprocesses

try:
    start_world()

    print("Starting position server...")
    pos_server = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && python3 ~/catkin_ws/src/opencv_tutorial/src/get_position_server.py"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(pos_server)

    print("Running pick-place node...")
    pick_place = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun cobot_IK pick_place_simple_client_node"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(pick_place)

    while pick_place.poll() is None:
        time.sleep(1)

    print("\nPick-place script finished. Shutting down all processes...")
    kill_all_ros_processes()

except KeyboardInterrupt:
    kill_all_ros_processes()  # Fully clean up before exit
