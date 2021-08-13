import rospy
from tubular_toolpath_server import TubularToolpathServer

rotation_begin = 30
rotation_end = 60
rotation_step = 10


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("tubular_toolpath_creator")
    # Go to the main loop.
    tubular_toolpath_server = TubularToolpathServer()
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()