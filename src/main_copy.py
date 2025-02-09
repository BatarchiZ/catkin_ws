import subprocess
import os
import time
import cv2
import sys

processes = []  # List to store subprocesses

def kill_all_ros_processes():
    """Forcefully terminate all ROS and Gazebo processes."""
    print("\nTerminating all processes and cleaning up ROS services...")

    subprocess.run(["pkill", "-f", "ros"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gazebo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # subprocess.run(["pkill", "-f", "python3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  # Kills any lingering Python ROS nodes
    process_list = subprocess.run(
        ["pgrep", "-f", "python3"],
        stdout=subprocess.PIPE,
        text=True
    ).stdout.strip().split("\n")
    print(process_list)

    for pid in process_list:
        if pid.isdigit():
            cmdline = subprocess.run(
                ["ps", "-p", pid, "-o", "cmd="],
                stdout=subprocess.PIPE,
                text=True
            ).stdout.strip()
            print(cmdline)

            if "gym_class" not in cmdline:
                subprocess.run(["kill", "-9", pid])

    subprocess.run(["pkill", "-f", "pick_place_simple_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    subprocess.run(["pkill", "-f", "object_tracker"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(2)

    subprocess.run(["killall", "-9", "rosmaster", "roslaunch", "gzserver", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    os.system("rosnode kill -a")  # Kill all active nodes
    os.system("pkill -f ros")  # Kill any lingering ROS-related processes
    os.system("pkill -f gazebo")  # Kill Gazebo
    os.system("pkill -9 gzserver")  # Kill Gazebo server
    os.system("pkill -9 gzclient")  # Kill Gazebo client

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

# def move_cobot(x, y, z):
#     print(f"Executing pick-and-place at position: x={x}, y={y}, z={z}")

#     pick_place = subprocess.Popen(
#         ["bash", "-c", f"source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
#         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid
#     )
#     processes.append(pick_place)

#     while pick_place.poll() is None:
#         time.sleep(1)

#     print("\nPick-place script finished.")
#     return 0


import signal

# def move_cobot(x, y, z, log_file="/home/is/catkin_ws/src/____logs/cobot_log.txt"):
#     print(f"Executing pick-and-place at position: x={x}, y={y}, z={z}")

#     with open(log_file, "a", buffering=1) as log:  # Open log file in line-buffered mode
#         # try:
#             pick_place = subprocess.Popen(
#                 ["bash", "-c", f"export PYTHONUNBUFFERED=1; source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && stdbuf -oL rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 text=True,
#                 bufsize=1
#             )

#             while True:
#                 output = pick_place.stdout.readline()
#                 if output == "" and pick_place.poll() is not None:
#                     break
#                 if output:
#                     sys.stdout.write(output)
#                     sys.stdout.flush()
#                     log.write(output)
#                     log.flush()  # Force write to file

#             while True:
#                 error = pick_place.stderr.readline()
#                 if error == "" and pick_place.poll() is not None:
#                     break
#                 if error:
#                     sys.stderr.write(error)
#                     sys.stderr.flush()
#                     log.write(error)
#                     log.flush()  # Force write to file

#             pick_place.wait()  # Wait for process completion

#     print("\nPick-place script finished. Log saved to", log_file)
#     return 0

import subprocess
import sys
import time

def move_cobot(x, y, z, log_file="/home/is/catkin_ws/src/____logs/cobot_log.txt", timeout=120):
    print(f"Executing pick-and-place at position: x={x}, y={y}, z={z}")

    with open(log_file, "a", buffering=1) as log:  # Open log file in line-buffered mode
        try:
            pick_place = subprocess.Popen(
                ["bash", "-c", f"export PYTHONUNBUFFERED=1; source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && stdbuf -oL rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            start_time = time.time()

            while True:
                # Check for timeout
                if time.time() - start_time > timeout:
                    print("\nTimeout reached! Terminating the pick-and-place process...")
                    pick_place.terminate()
                    try:
                        pick_place.wait(timeout=5)  # Allow process to clean up
                    except subprocess.TimeoutExpired:
                        pick_place.kill()  # Force kill if still running
                    return -1  # Indicate timeout

                output = pick_place.stdout.readline()
                if output == "" and pick_place.poll() is not None:
                    break
                if output:
                    sys.stdout.write(output)
                    sys.stdout.flush()
                    log.write(output)
                    log.flush()  # Force write to file

                error = pick_place.stderr.readline()
                if error:
                    sys.stderr.write(error)
                    sys.stderr.flush()
                    log.write(error)
                    log.flush()  # Force write to file

            pick_place.wait()  # Wait for process completion

        except Exception as e:
            print(f"Error: {e}")
            return -1  # Return error code on failure

    print("\nPick-place script finished. Log saved to", log_file)
    return 0  # Indicate success


if __name__ == "__main__":
    processes = []  # List to store subprocesses

    try:
        start_world()

        coords = [(0.0, 0.0, 0.5), (1.0, 0.0, 0.5)]
        for coord in coords:
            path = "/home/is/catkin_ws/src/z_output/recent_frame.jpg"
            cur_camera_image = cv2.imread(path)
            print(cur_camera_image.shape)
            x, y, z = coord  # Example target position
            move_cobot(x, y, z)

        kill_all_ros_processes()

    except KeyboardInterrupt:
        kill_all_ros_processes()  # Fully clean up before 