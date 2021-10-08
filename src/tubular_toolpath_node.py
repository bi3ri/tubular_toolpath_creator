#!/usr/bin/env python
import rospy

from tubular_toolpath_creator.tubular_toolpath_server import TubularToolpathServer

ply_path = "/home/bi3ri/hotspray_ws/src/tubular_toolpath_creator/data/original/coil_scan00.ply"
ply_path1 = "/home/bi3ri/hotspray_ws/src/hotspray/hotspray_application/data/meshs/results_mesh.ply"


if __name__ == "__main__":
    rospy.init_node("tubular_toolpath_creator")
    tubular_toolpath_server = TubularToolpathServer()
    # while(1):
    #     tubular_toolpath_server.run(ply_path1)
    rospy.spin()