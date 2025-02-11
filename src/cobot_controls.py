import subprocess
import time
import select

def control_gripper(position, duration=1.0):
    """
    Control the gripper by sending a terminal command.
    
    :param position: Target gripper position (e.g., 0.0 = closed, 0.5 = half-open, 1.0 = fully open)
    :param duration: Time (in seconds) for the movement to complete
    """
    command = f"""rostopic pub -1 /cobot/gripper_controller/command trajectory_msgs/JointTrajectory '
joint_names: ["gripper_right_joint"]
points:
- positions: [{position}]
  time_from_start: {{secs: {int(duration)}, nsecs: 0}}'"""
    
    # print(f"Executing: {command}")  # Debugging output
    subprocess.run(command, shell=True)


def move_cobot(x, y, z, log_file="/home/is/catkin_ws/src/____logs/cobot_log.txt", timeout=130):
    print(f"Executing pick-and-place at position: x={x}, y={y}, z={z}")
    with open(log_file, "a", buffering=1) as log:  # Line-buffered mode
        try:
            pick_place = subprocess.Popen(
                ["bash", "-c", f"export PYTHONUNBUFFERED=1; source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && stdbuf -oL rosrun cobot_IK _pick_place_argpass_node {x} {y} {z}"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1  # Line-buffered output
            )

            start_time = time.time()

            while True:
                # Timeout check
                if time.time() - start_time > timeout:
                    print("\nTimeout reached! Terminating the pick-and-place process...")
                    pick_place.terminate()
                    try:
                        pick_place.wait(timeout=5)  # Allow graceful shutdown
                    except subprocess.TimeoutExpired:
                        pick_place.kill()  # Force kill if still running
                    return -1  # Indicate timeout

                # Use `select` to check if there's new output **without blocking**
                ready_to_read, _, _ = select.select([pick_place.stdout, pick_place.stderr], [], [], 0.1)

                for stream in ready_to_read:
                    line = stream.readline()
                    if line:
                        # sys.stdout.write(line)
                        # sys.stdout.flush()
                        log.write(line)
                        log.flush()

                # If process is done, exit loop
                if pick_place.poll() is not None:
                    break

            pick_place.wait()  # Ensure process fully exits
            return 0  # Success

        except Exception as e:
            print(f"Error: {e}")
            return -1  # Return error code on failure