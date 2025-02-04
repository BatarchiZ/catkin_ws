import subprocess
import os
import time
import cv2

def kill_all_ros_processes():
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

def start_world():
    print("Launching robot simulation...")
    sim_process = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch cobot_sim spawn_robot_2Dcamera_traj_controller_world.launch"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(sim_process)

    camera_process = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && python3 /home/is/catkin_ws/src/camera_publisher.py"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(camera_process)
    time.sleep(5) 
    
    return 0

def move_cobot(x, y, z):
    print(f"Executing pick-and-place at position: x={x}, y={y}, z={z}")

    pick_place = subprocess.Popen(
        ["bash", "-c", f"source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
    )
    processes.append(pick_place)

    while pick_place.poll() is None:
        time.sleep(1)

    print("\nPick-place script finished.")
    return 0

if __name__ == "__main__":
    processes = []  # List to store subprocesses

    try:
        start_world()

        coords = [(1.0, 0.0, 0.5), (0.0, 1.0, 0.5)]
        for coord in coords:
            path = "/home/is/catkin_ws/src/z_output/recent_frame.jpg"
            cur_camera_image = cv2.imread(path)
            print(cur_camera_image.shape)
            x, y, z = coord  # Example target position
            move_cobot(x, y, z)

        kill_all_ros_processes()

    except KeyboardInterrupt:
        kill_all_ros_processes()  # Fully clean up before 