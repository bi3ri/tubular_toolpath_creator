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
from tubular_toolpath_creator.debug_visualisation import DebugPoses

DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 
DEBUG = True

class TubularToolpathServer:

    def __init__(self):
        self.rot_begin = rospy.get_param('~rotation_begin') 
        self.rot_end = rospy.get_param('~rotation_end')
        self.rot_step = rospy.get_param('~rotation_step')
        self.z_clip_height = rospy.get_param('~z_clip_height') 
        self.voxel_down_sample_size = rospy.get_param('~voxel_down_sample_size') 
        self.centerline_target_reduction = rospy.get_param('~centerline_target_reduction') 
        self.toolpath_segment_point_reduction = rospy.get_param('~toolpath_segment_point_reduction') 
        self.smoothing_mesh_factor = rospy.get_param('~smoothing_mesh_factor') 
        self.pose_spacing = rospy.get_param('~pose_spacing') 
        self.frame_id = rospy.get_param('~frame_id')
        self.debug = rospy.get_param('~debug')

        self.service = rospy.Service('tubular_toolpath_creator/create_tubular_toolpath', GenerateTubularToolpath, self.handle_request)
        self.publisher = rospy.Publisher('tubular_toolpath_raster', MarkerArray, queue_size=100)

        self.debug_poses = DebugPoses('tubular_toolpath')
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
        left_mesh = clipper_middle.GetOutput(1)

        if self.debug:
            saveVtp(DATA_PATH + '/debug/right_mesh.vtp', right_mesh)  #debug
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

    def createDirectionalVectors(self, p1, p2, rot_center):
        vx_norm = normalize(p2 - p1)
        p1_rot_center = rot_center - p1
        vy_norm = normalize(np.cross(vx_norm, p1_rot_center))
        vz_norm = normalize(np.cross(vy_norm, vx_norm))
        return vx_norm, vy_norm, vz_norm

    def createRotationSegment(self, coil_segment, cut_normal, rot_center, centerline_segment_middle):
        #cut front of tubular surface relative to rot_center in order to not cut surface twice
        clip = vtk.vtkClipPolyData()
        clip_plane = vtk.vtkPlane()
        clip_plane.SetNormal(rot_center - centerline_segment_middle)
        clip_plane.SetOrigin(centerline_segment_middle) 
        clip.SetInputData(coil_segment) 
        clip.SetClipFunction(clip_plane)
        clip.Update()
        
        cut_plane = vtk.vtkPlane()
        cut_plane.SetNormal(cut_normal)
        cut_plane.SetOrigin(rot_center)

        cutter = vtk.vtkCutter()
        cutter.SetCutFunction(cut_plane)
        cutter.SetGenerateTriangles(False)
        cutter.SetSortBy(1)
        cutter.SetNumberOfContours(1)
        cutter.SetInputConnection(clip.GetOutputPort())
        cutter.Update()
        
        strip_one = vtk.vtkStripper()
        strip_one.SetInputData(cutter.GetOutput())
        strip_one.JoinContiguousSegmentsOn()
        strip_one.Update()

        decimateFilter = vtk.vtkDecimatePolylineFilter()
        decimateFilter.SetTargetReduction(0.001)
        decimateFilter.SetInputData(strip_one.GetOutput())
        decimateFilter.Update()

        return decimateFilter.GetOutput()

    def createRotationSegmentPoseArray(self, segments, toolpath_direction, rot_centers):
        start_id = None
        end_id = None

        if toolpath_direction == "left":
            points = segments[0].GetPoints()

            start_id = 1
            end_id = points.GetNumberOfPoints()
            first_id = 0
            last_id = end_id - 1
        else:
            segments.reverse()
            rot_centers.reverse()
            points = segments[0].GetPoints()

            start_id = points.GetNumberOfPoints() - 1
            end_id = -1
            first_id = start_id - 1
            last_id = 0

        p1 = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        p2 = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        vx_norm = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        vy_norm = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        vz_norm = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)

        p1[0] = points.GetPoint(first_id)[0]
        p1[1] = points.GetPoint(first_id)[1]
        p1[2] = points.GetPoint(first_id)[2]

        for j in range(len(segments)):
            points = segments[j].GetPoints()
            increment = 0

            if toolpath_direction == "left":
                start_id = 0
                end_id = points.GetNumberOfPoints() # - 1
                last_id = end_id - 1
                increment = 1
            else:
                start_id = points.GetNumberOfPoints() - 1
                end_id = -1 # 0
                last_id = 0
                increment = -1

            for i in range(start_id, end_id, increment):
                p2[0] = points.GetPoint(i)[0]
                p2[1] = points.GetPoint(i)[1]
                p2[2] = points.GetPoint(i)[2]

                if euclideanDistancePose(p1, p2) < self.pose_spacing: continue
                    
                vx_norm, vy_norm, vz_norm = self.createDirectionalVectors(p1, p2, rot_centers[j])
                if self.debug: self.debug_poses.addPose(p1, vx_norm, vy_norm, vz_norm)
                self.appendPose(p1, vx_norm, vy_norm, vz_norm)
                p1 = copy.deepcopy(p2)

        # last p1 gets orientation from point before rotation
        p = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
        p[0] = float(points.GetPoint(last_id)[0])
        p[1] = float(points.GetPoint(last_id)[1])
        p[2] = float(points.GetPoint(last_id)[2])
        self.appendPose(p, vx_norm, vy_norm, vz_norm)

    def createToolpath(self, center_line_points, mesh_segments):
        toolpath_direction = "right"
        center = self.findCenterOfCoil(center_line_points)
        center_line_size = center_line_points.shape[0]

        for r in range(self.rot_begin, self.rot_end, self.rot_step):
            gap_filter = GapFilter(center_line_size)
            rot_centers = []

            for i in range(0, center_line_size - 1):
                axis = center_line_points[i + 1] - center_line_points[i] 

                #translate roation_center relative to coordinate origin, rotate and translate back
                translation_center = center - center_line_points[i] 
                rot_center = rotatePointAroundAxis(translation_center, axis, r)
                rot_center += center_line_points[i]
                rot_centers.append(rot_center)

                middle = (center_line_points[i+1] - center_line_points[i])/2 + center_line_points[i]
                cut_normal = np.cross(center_line_points[i+1] - rot_center, center_line_points[i] - rot_center)

                #cut surface to create toolpath segment
                rotation_segement = self.createRotationSegment(mesh_segments[i], cut_normal, rot_center, middle)

                gap_filter.addSegment(rotation_segement)

            if self.debug:
                combined_rotation_segment = gap_filter.getCombinedRotationSegement()
                saveVtp(os.path.join(DATA_PATH, ('debug/combined_rotation_segment/combined_rotation_segment_degree_' + str(r) + '.vtp')), combined_rotation_segment)

            ### create toolpath poses
            segments = gap_filter.getSegments()
            self.createRotationSegmentPoseArray(segments, toolpath_direction, rot_centers)
            toolpath_direction = "right" if toolpath_direction == "left" else "left"
            gap_filter = None

    def computeCenterline(self, input_mesh_path, output_centerline_path):
        script_path = os.path.normpath( os.path.join(os.path.dirname(__file__), 'conda/toolpath_centerline.bash'))
        execute_string =  script_path + ' ' + input_mesh_path + ' ' + output_centerline_path
        rc = subprocess.call(execute_string, shell=True)

    def run(self, ply_path):
        # fill gaps and create mesh with open3d
        rospy.loginfo('Loading pointcload and closing gaps.')
        watertight_stl_path = os.path.join(DATA_PATH, 'tmp/watertight_coil.stl')
        cropAndFillGapsInMesh(ply_path, watertight_stl_path, 0.01, self.voxel_down_sample_size)#, self.debug)

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
        centerline = reducePolylinePointResolution(centerline_source, self.centerline_target_reduction)
        centerline_points = (pv.wrap(centerline)).points #delete pv
        rospy.loginfo('Number of centerline points: %i', centerline_points.shape[0])

        #split mesh in segments
        mesh_segments = self.splitMeshInSegments(centerline_points, clipped_mesh)

        #create toolpath poses
        self.createToolpath(centerline_points, mesh_segments)

        if self.debug:
            self.debug_poses.saveVtp(DATA_PATH)
            idx = 0
            for segment in mesh_segments:
                saveVtp(os.path.join(DATA_PATH , ('debug/mesh_segments/mesh_segment' + str(idx) + '.vtp')), segment)
                idx+=1

        marker_array = convertDirectionVectorPoseToAxisMarkers(self.pose_array, self.frame_id)
        self.publisher.publish(marker_array)

        rospy.loginfo('Created toolpath')

    def handle_request(self, req):
        self.run(req.mesh_path)
        response = GenerateTubularToolpathResponse()
        response_array = Float64MultiArray()
        response_array.data = self.pose_array
        response.toolpath_vector_array = response_array
        return response


# server = TubularToolpathServer()
# ply_path = os.path.join(DATA_PATH, 'original/coil_scan.ply')
# server.run(ply_path)
# # server.debug_line.render()

# server.debug_line.save('debug_test')
#https://wiki.ogre3d.org/Quaternion+and+Rotation+Primer
