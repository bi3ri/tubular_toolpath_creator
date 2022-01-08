#!/usr/bin/env python
import rospy
import sys

from tubular_toolpath_creator.tubular_toolpath_server import TubularToolpathServer

if __name__ == "__main__":
    rospy.init_node("tubular_toolpath_creator")
    mesh_path = rospy.get_param('~mesh_path')

    tubular_toolpath_server = TubularToolpathServer()
    if mesh_path:
        tubular_toolpath_server.run(mesh_path)
    
    rospy.spin()