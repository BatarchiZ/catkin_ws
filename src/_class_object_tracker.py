import rospy
from gazebo_msgs.msg import ModelStates
import time

class GazeboObjectTracker:
    def __init__(self):
        print("here0")
        rospy.init_node(f"object_tracker{str(time.time())}", anonymous=True)
        print("here1")
        self.model_states = None
        self.last_update_time = rospy.Time.now()
        print("here2")
        
        # Subscribe to Gazebo model states
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        print("here3")
        rospy.sleep(1)  # Allow time for messages to be received
        print("here4")

    def callback(self, data):
        """Throttled callback to store the latest object positions."""
        current_time = rospy.Time.now()
        if (current_time - self.last_update_time).to_sec() < 0.1:  # Process every 100ms
            return
        self.last_update_time = current_time
        
        self.model_states = data  # Store latest message

    def get_object_position(self, object_name):
        """Retrieves the latest position of the object."""
        if self.model_states is None:
            rospy.logwarn("No model states received yet.")
            return None

        try:
            index = self.model_states.name.index(object_name)
            position = self.model_states.pose[index].position
            return position.x, position.y, position.z
        except ValueError:
            rospy.logerr(f"Object {object_name} not found in Gazebo!")
            return None
        
    def shutdown(self):
        print("shutting down node")
        rospy.signal_shutdown("Restart")


if __name__ == "__main__":
    tracker = GazeboObjectTracker()
    
    rospy.sleep(2)  # Ensure the subscriber gets at least one message
    
    obj_name = "object_pick"
    position = tracker.get_object_position(obj_name)
    
    if position:
        print(f"{obj_name} Position: {position}")
    else:
        print("Failed to get object position.")

    rospy.spin()  # Keep the node alive
