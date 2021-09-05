#!/usr/bin/env python
import rospy

from tubular_toolpath_creator.tubular_toolpath_server import TubularToolpathServer

if __name__ == "__main__":
    rospy.init_node("tubular_toolpath_creator")
    tubular_toolpath_server = TubularToolpathServer()
    rospy.spin()