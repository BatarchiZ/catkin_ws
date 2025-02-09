#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger, TriggerResponse
import time

class GazeboObjectTracker:
    def __init__(self):
        rospy.init_node(f"object_tracker_", anonymous=True)

        self.model_states = None
        self.last_update_time = rospy.Time.now()

        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.service = rospy.Service("/get_object_position", Trigger, self.get_position_callback)

        rospy.sleep(1)  # Allow time for messages to be received

    def callback(self, data):
        self.last_update_time = rospy.Time.now()
        self.model_states = data  # Store latest message

    def get_position_callback(self, request):
        """ROS service to return the position of an object"""
        object_name = "object_pick"  # Change this if you want dynamic object names

        if self.model_states is None:
            rospy.logwarn("No model states received yet.")
            return TriggerResponse(success=False, message="No model states received.")

        try:
            index = self.model_states.name.index(object_name)
            position = self.model_states.pose[index].position
            return TriggerResponse(success=True, message=f"{position.x}, {position.y}, {position.z}")
        except ValueError:
            return TriggerResponse(success=False, message=f"Object {object_name} not found.")

if __name__ == "__main__":
    tracker = GazeboObjectTracker()
    rospy.spin()  # Keep the node alive
