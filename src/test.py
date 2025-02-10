import rospy
import tf
import numpy as np
from __class_gym import get_object_position, launch_tracker

# Object size from SDF
OBJECT_SIZE_X = 0.02  # X dimension
OBJECT_SIZE_Y = 0.02  # Y dimension
OBJECT_SIZE_Z = 0.2   # Z dimension (height)

def get_tf_position(listener, parent_frame, child_frame):
    """ Get the position of a TF frame relative to another. """
    try:
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        return np.array(trans)  # Returns (x, y, z) as a NumPy array
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None

def compute_midpoint(base_position):
    """ Compute the actual midpoint of `object_pick` using its dimensions. """
    if base_position is None:
        return None
    
    # Midpoint is at half the height (Z direction)
    midpoint = np.copy(base_position)
    midpoint[2] += OBJECT_SIZE_Z / 2  # Move Z up by half the height
    return midpoint

def main():
    launch_tracker()

    rospy.init_node('gripper_object_distance_calculator')
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Get Gripper Midpoint Position
        gripper_midpoint = get_tf_position(listener, "world", "grippers_midpoint")
        # print(gripper_midpoint)
        
        # Get Object Base Position (not the midpoint yet)
        object_pick_base = get_object_position()
        # print(object_pick_base)

        if gripper_midpoint is not None and object_pick_base is not None:
            # Compute the true midpoint of the object
            object_pick_midpoint = compute_midpoint(object_pick_base)
            
            # Calculate Euclidean Distance
            distance = np.linalg.norm(gripper_midpoint - object_pick_midpoint)
            
            print(f"Gripper Midpoint: {gripper_midpoint}")
            print(f"Object Pick Midpoint: {object_pick_midpoint}")
            print(f"Euclidean Distance: {distance:.4f} meters\n")

        rate.sleep()

if __name__ == "__main__":
    main()
