#!/usr/bin/env python
import rospy
import os

from tubular_toolpath_creator.tubular_toolpath_server import TubularToolpathServer
data_path = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 


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


# server = TubularToolpathServer()
# ply_path = os.path.join(data_path, 'original/coil_scan.ply')
# server.run(ply_path)
# # server.debug_line.render()
# server.debug_x.saveVtp(data_path)
# server.debug_y.saveVtp(data_path)
# server.debug_z.saveVtp(data_path)

# server.debug_line.save('debug_test')
