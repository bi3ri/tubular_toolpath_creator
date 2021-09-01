#!/usr/bin/env python
import rospy
import os
import subprocess
import copy

import pyvista as pv
from spatialmath import *

import vtk
import numpy as np
from std_msgs.msg import Float64MultiArray

from tubular_toolpath_creator.gap_filter import GapFilter
from tubular_toolpath_creator.utils import *
from tubular_toolpath_creator.srv import GenerateTubularToolpath, GenerateTubularToolpathResponse
from tubular_toolpath_creator.debug_lines import DebugLines

DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 
DEBUG = True


class TubularToolpathServer:
    nodeHandle = None
    plotter = pv.Plotter()
    smoothing_mesh_factor = 5000
    pose_spacing = 0.009
    # colors = vtk.vtkNamedColors() 
    rot_begin = 45
    rot_end = 46
    rot_step = 11
    service = None

    def __init__(self):
        # self.rot_begin = rospy.get_param('~tubular_toolpath_creator', 'rotation_begin') #hier parameter von nodeHandle
        # self.rot_end = rospy.get_param('~tubular_toolpath_creator', 'rotation_end')
        # self.rot_step = rospy.get_param('~tubular_toolpath_creator', 'rotation_step')
        self.z_clip_height = 0.1
        self.service = rospy.Service('tubular_toolpath_creator/create_tubular_toolpath', GenerateTubularToolpath, self.handle_request)
        self.centerlineTargetReduction = 0.93 #0.95
        self.toolpathSegmentPointResolution = 0.35
        # self.publisher = rospy.Publisher('tubular_toolpath_raster', MarkerArray, queue_size=100)
        self.publisher = rospy.Publisher('tubular_toolpath_raster', MarkerArray, queue_size=100)
        self.debug_x = DebugLines('x')
        self.debug_y = DebugLines('y')
        self.debug_z = DebugLines('z')
        self.pose_array = []

    def findCenterOfCoil(self, center_line_points):
        center = np.mean(center_line_points, axis=0)
        center[2] = center_line_points[0][2] 
        return center

    def splitMeshInSegments(self, center_line_points, mesh):
        center_line_size = center_line_points.shape[0]
        middle = center_line_size // 2
        clipped_coil_array = [None] * (center_line_size - 1)
        clip_plane_middle = vtk.vtkPlane()
        normal = center_line_points[middle +1] - center_line_points[middle]
        clip_plane_middle.SetNormal(normal)
        clip_plane_middle.SetOrigin(center_line_points[middle]) 

        clipper_middle = vtk.vtkClipPolyData()
        clipper_middle.SetGenerateClippedOutput(True)
        clipper_middle.SetInputData(mesh)
        clipper_middle.SetClipFunction(clip_plane_middle)
        clipper_middle.Update()

        right_mesh = clipper_middle.GetOutput(0)
        saveVtp(DATA_PATH + '/debug/right_mesh.vtp', right_mesh)  #debug
        left_mesh = clipper_middle.GetOutput(1)
        saveVtp(DATA_PATH + '/debug/left_mesh.vtp', left_mesh) #debug

        remaining_mesh = left_mesh
        for i in range(1, middle):
            clip = vtk.vtkClipPolyData()
            clip.SetGenerateClippedOutput(True)

            clip_plane = vtk.vtkPlane()
            normal = center_line_points[i+1] - center_line_points[i]
            clip_plane.SetNormal(normal)
            clip_plane.SetOrigin(center_line_points[i]) 
            clip.SetInputData(remaining_mesh) 
            clip.SetClipFunction(clip_plane)
            clip.Update()

            clipped_coil_array[i - 1] = clip.GetOutput(1)
            remaining_mesh = clip.GetOutput(0)

        clipped_coil_array[middle - 1] = remaining_mesh

        remaining_mesh = right_mesh
        for i in range(middle + 1, center_line_size - 1):
            clip = vtk.vtkClipPolyData()
            clip.SetGenerateClippedOutput(True)

            clip_plane = vtk.vtkPlane()
            normal = center_line_points[i+1] - center_line_points[i]
            clip_plane.SetNormal(normal)
            clip_plane.SetOrigin(center_line_points[i]) 
            clip.SetInputData(remaining_mesh) 
            clip.SetClipFunction(clip_plane)
            clip.Update()

            clipped_coil_array[i - 1] = clip.GetOutput(1)
            remaining_mesh = clip.GetOutput(0)

        clipped_coil_array[center_line_size - 2] = remaining_mesh
        
        return clipped_coil_array

    def appendPose(self, p1, vx_norm, vy_norm, vz_norm):
        self.pose_array.append(float(p1[0]))
        self.pose_array.append(float(p1[1]))
        self.pose_array.append(float(p1[2]))
        self.pose_array.append(float(vx_norm[0]))
        self.pose_array.append(float(vx_norm[1]))
        self.pose_array.append(float(vx_norm[2]))
        self.pose_array.append(float(vy_norm[0]))
        self.pose_array.append(float(vy_norm[1]))
        self.pose_array.append(float(vy_norm[2]))
        self.pose_array.append(float(vz_norm[0]))
        self.pose_array.append(float(vz_norm[1]))
        self.pose_array.append(float(vz_norm[2]))

    def createRotationSegment(self, coil_segment, cut_normal, rot_center, centerline_segment_middle):
        #cut front of tubular surface relative to rot_center in order to not cut surface twice
        clip = vtk.vtkClipPolyData()
        # clip.SetGenerateClippedOutput(True)
        clip_plane = vtk.vtkPlane()
        clip_plane.SetNormal(rot_center - centerline_segment_middle)
        clip_plane.SetOrigin(centerline_segment_middle) 
        clip.SetInputData(coil_segment) 
        clip.SetClipFunction(clip_plane)
        clip.Update()
        
        cut_plane = vtk.vtkPlane()
        # cut_normal = np.cross()
        cut_plane.SetNormal(cut_normal)
        cut_plane.SetOrigin(rot_center)

        cutter = vtk.vtkCutter()
        cutter.SetCutFunction(cut_plane)
        cutter.SetGenerateTriangles(False)
        cutter.SetSortBy(1)
        cutter.SetNumberOfContours(1)
        cutter.SetInputConnection(clip.GetOutputPort())
        cutter.Update()
        rotation_segement = cutter.GetOutput()
        
        # TODO: not sure if i need the next two blocks
        strip_one = vtk.vtkStripper()
        strip_one.SetInputData(rotation_segement)
        strip_one.JoinContiguousSegmentsOn()
        # strip.SetMaximumLength(1)
        strip_one.Update()
        # output = strip_one.GetOutput()

        decimateFilter2 = vtk.vtkDecimatePolylineFilter()
        decimateFilter2.SetTargetReduction(0.7)
        # decimateFilter.SetMaximumError(0.3)
        decimateFilter2.SetInputData(strip_one.GetOutput())
        decimateFilter2.Update()
        rotation_segement = decimateFilter2.GetOutput()

        return rotation_segement

    def createRotationSegmentPoseArray(self, combined_rotation_segment, toolpath_direction, rot_center):
        combined_rotation_segment_points = combined_rotation_segment.GetPoints()

        start_id = 0
        end_id = 0

        if toolpath_direction == "right":
            start_id = 1
            end_id = combined_rotation_segment_points.GetNumberOfPoints() - 1
            first_id = 0
            toolpath_direction = "left"
        else:
            start_id = combined_rotation_segment_points.GetNumberOfPoints() - 1
            end_id = 0
            first_id = start_id - 1
            toolpath_direction = "right"

        #start p1
        p1 = [0, 0, 0]
        p2 = [0, 0, 0]
        p2[0] = combined_rotation_segment_points.GetPoint(first_id)[0]
        p2[1] = combined_rotation_segment_points.GetPoint(first_id)[1]
        p2[2] = combined_rotation_segment_points.GetPoint(first_id)[2]
            
        for i in range(start_id, end_id):
            p1 = copy.deepcopy(p2)
            p2[0] = combined_rotation_segment_points.GetPoint(i)[0]
            p2[1] = combined_rotation_segment_points.GetPoint(i)[1]
            p2[2] = combined_rotation_segment_points.GetPoint(i)[2]


            if euclideanDistancePose(p1, p2) < self.pose_spacing:
                continue

            # p1 -> rot_center
            vz_norm = normalize(rot_center - [p1.position.x, p1.position.y, p1.position.z])

            #p1 -> p2
            toolpath_direction = normalize(np.array([p2.position.x, p2.position.y, p2.position.z])
                                        - np.array([p1.position.x, p1.position.y, p1.position.z]))

            pose_vis = [p1.position.x, p1.position.y, p1.position.z]

            self.debug_z.addLine(pose_vis, vz_norm+pose_vis)

            vy_norm = normalize(np.cross(toolpath_direction, vz_norm))
            self.debug_y.addLine(pose_vis, vy_norm+pose_vis)

            vx_norm = normalize(np.cross(vz_norm, vy_norm))

            vy_norm = -vy_norm 

            self.debug_x.addLine(pose_vis, vx_norm+pose_vis)

            self.appendPose(p1, vx_norm, vy_norm, vz_norm)

            #remove outliers
            if p1.orientation.x == 0 or p1.orientation.y == 0 or p1.orientation.z == 0 or p1.orientation.w == 0:
                print('orientation corrupt')
            #     continue 

        #last p1 gets orientation from point before rotation

        p1[0] = float(combined_rotation_segment_points.GetPoint(end_id)[0])
        p1[1] = float(combined_rotation_segment_points.GetPoint(end_id)[1])
        p1[2] = float(combined_rotation_segment_points.GetPoint(end_id)[2])
        self.appendPose(p1, vx_norm, vy_norm, vz_norm)

        return toolpath_direction

    def createToolpath(self, center_line_points, mesh_segments):
        toolpath_direction = "right"

        center = self.findCenterOfCoil(center_line_points)
        center_line_size = center_line_points.shape[0]

        for r in range(self.rot_begin, self.rot_end, self.rot_step):
            gap_filter = GapFilter(center_line_size)

            for i in range(0, center_line_size - 1):
                axis = center_line_points[i + 1] - center_line_points[i] 
                #axis_norm = axis / np.linalg.norm(axis)

                #translate roation_center relative to coordinate ursprung, rotate and translate back
                translation_center = center - center_line_points[i] 
                rot_center = rotatePointAroundAxis(translation_center, axis, r)
                rot_center += center_line_points[i]

                middle = (center_line_points[i+1] - center_line_points[i])/2 + center_line_points[i]
                cut_normal = np.cross(center_line_points[i+1] - rot_center, center_line_points[i] - rot_center)

                #cut surface to create toolpath segment
                rotation_segement = self.createRotationSegment(mesh_segments[i], cut_normal, rot_center, middle)
                # renderVtkPolydata(rotation_segement)

                #closing gaps between two cut plane segments
                reduced_rotation_segement = reducePolylinePointResolution(rotation_segement, self.toolpathSegmentPointResolution)
                gap_filter.addSegment(reduced_rotation_segement)

    
            combined_rotation_segment = gap_filter.getCombinedRotationSegement()
            gap_filter = None
            saveVtp(DATA_PATH + '/debug/combined_rotation_segment_degree_' + str(r) + '.vtp', combined_rotation_segment)
            # saveVtp(os.path.join(DATA_PATH, 'cut_kaputt.vtp'), combined_rotation_segment)

            # renderVtkPolydata(combined_rotation_segment)

            ### create toolpath poses
            toolpath_direction = self.createRotationSegmentPoseArray(combined_rotation_segment, toolpath_direction, rot_center)
            # toolpath_raster.append(combined_rotation_segment_pose_array)

        # return toolpath_raster

    def computeCenterline(self, input_mesh_path, output_centerline_path):
        script_path = os.path.normpath( os.path.join(os.path.dirname(__file__), 'conda/toolpath_centerline.bash'))
        execute_string =  script_path + ' ' + input_mesh_path + ' ' + output_centerline_path
        rc = subprocess.call(execute_string, shell=True)

    def run(self, ply_path):
        # fill gaps and create mesh with open3d
        watertight_stl_path = os.path.join(DATA_PATH, 'tmp/watertight_coil.stl')
        # cropAndFillGapsInMesh(ply_path, watertight_stl_path, 0.01)

        # smooth mesh
        watertight_mesh = loadStl(watertight_stl_path) 
        smoothed_mesh = smoothMesh(watertight_mesh, 100)

        # clip mesh
        clipped_mesh = clipMeshAtZaxis(smoothed_mesh, self.z_clip_height)
        clipped_vtp_path = os.path.join(DATA_PATH, 'tmp/clipped_coil.vtp')
        saveVtp(clipped_vtp_path, clipped_mesh)
        
        # compute centerline
        centerline_path = os.path.join(DATA_PATH, 'tmp/centerline.vtp')
        self.computeCenterline(clipped_vtp_path, centerline_path)
        centerline_source = loadVtp(centerline_path)
        centerline = reducePolylinePointResolution(centerline_source, self.centerlineTargetReduction)
        centerline_points = (pv.wrap(centerline)).points #delete pv
        print('Number of centerline points: ' + str(centerline_points.shape[0]))

        #split mesh in segments
        mesh_segments = self.splitMeshInSegments(centerline_points, clipped_mesh)

        #create toolpath poses
        # toolpath_raster = 
        self.createToolpath(centerline_points, mesh_segments)

        idx = 0
        if DEBUG:
            for segment in mesh_segments:
                saveVtp(DATA_PATH + '/debug/mesh_segment' + str(idx) + '.vtp', segment)
                idx+=1

        # self.publisher.publish(toolpath_markers)
        print("created toolpath")

        self.debug_x.saveVtp(DATA_PATH)
        self.debug_y.saveVtp(DATA_PATH)
        self.debug_z.saveVtp(DATA_PATH)

        # return toolpath_raster

    def handle_request(self, req):
        # toolpath_raster = self.run(req.mesh_path)
        self.run(req.mesh_path)

        response = GenerateTubularToolpathResponse()
        # response.toolpath_raster = toolpath_raster
        response_array = Float64MultiArray()
        response_array.data = self.pose_array
        response.toolpath_vector_array = response_array
        return response


# server = TubularToolpathServer()
# ply_path = os.path.join(DATA_PATH, 'original/coil_scan.ply')
# server.run(ply_path)
# # server.debug_line.render()
# server.debug_x.saveVtp(DATA_PATH)
# server.debug_y.saveVtp(DATA_PATH)
# server.debug_z.saveVtp(DATA_PATH)

# server.debug_line.save('debug_test')
#https://wiki.ogre3d.org/Quaternion+and+Rotation+Primer