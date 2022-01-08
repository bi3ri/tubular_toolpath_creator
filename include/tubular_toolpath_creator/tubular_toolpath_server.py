#!/usr/bin/env python
import rospy
import os
import subprocess
import copy
from timeit import default_timer as timer

import pyvista as pv
# from spatialmath import *

import vtk
import numpy as np
from std_msgs.msg import Float64MultiArray, Int64
from geometry_msgs.msg import PoseArray, Pose

from tubular_toolpath_creator.gap_filter import GapFilter
from tubular_toolpath_creator.utils import *
from tubular_toolpath_creator.srv import GenerateTubularToolpath, GenerateTubularToolpathResponse
from tubular_toolpath_creator.debug_visualisation import *

DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 
DEBUG = True

class TubularToolpathServer:

    def __init__(self):
        self.rot_begin = rospy.get_param('~rotation_begin') 
        self.rot_end = rospy.get_param('~rotation_end')
        self.rot_step = rospy.get_param('~rotation_step')
        self.lower_rotation_end = rospy.get_param('~lower_rotation_end')
        self.lower_rotation_height = rospy.get_param('~lower_rotation_height')
        self.upper_spray_angle = rospy.get_param('~upper_spray_angle')
        self.z_clip_height = rospy.get_param('~z_clip_height') 
        self.voxel_down_sample_size = rospy.get_param('~voxel_down_sample_size') 
        self.centerline_target_reduction = rospy.get_param('~centerline_target_reduction') 
        self.centerline_minimal_point_size_treshold = rospy.get_param('~centerline_minimal_point_size_treshold') 
        self.toolpath_segment_point_reduction = rospy.get_param('~toolpath_segment_point_reduction') 
        self.smoothing_mesh_factor = rospy.get_param('~smoothing_mesh_factor') 
        self.pose_spacing = rospy.get_param('~pose_spacing')
        self.frame_id = rospy.get_param('~frame_id')
        self.debug = rospy.get_param('~debug')

        self.service = rospy.Service('tubular_toolpath_creator/create_tubular_toolpath', GenerateTubularToolpath, self.handle_request)
        self.publisher = rospy.Publisher('tubular_toolpath_raster', MarkerArray, queue_size=100)

        if self.debug:
            self.debug_poses = DebugPoses('tubular_toolpath')
            self.debug_rotation_points = DebugPoints()
            self.debug_rotation_vector = DebugLines("roation_vecotr", scaling_factor=0.03)

    def findCenterOfCoil(self, center_line_points):
        center = np.mean(center_line_points, axis=0)
        center[2] = center_line_points[0][2] 
        return center

    def splitMeshInSegments(self, center_line_points, mesh):
        center_line_size = len(center_line_points)
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
            saveVtp(DATA_PATH + '/debug/right_mesh.vtp', right_mesh)  
            saveVtp(DATA_PATH + '/debug/left_mesh.vtp', left_mesh) 

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

    def createRotationSegment(self, coil_segment, cut_normal, rotated_cut_direction, centerline_segment_middle, i, r):
        #cut front of tubular surface relative to rot_center in order to not cut surface twice
        clip = vtk.vtkClipPolyData()
        clip_plane = vtk.vtkPlane()
        clip_plane.SetNormal(rotated_cut_direction - centerline_segment_middle)
        clip_plane.SetOrigin(centerline_segment_middle) 
        clip.SetInputData(coil_segment) 
        clip.SetClipFunction(clip_plane)
        clip.Update()
        
        cut_plane = vtk.vtkPlane()
        cut_plane.SetNormal(cut_normal)
        cut_plane.SetOrigin(rotated_cut_direction)

        cutter = vtk.vtkCutter()
        cutter.SetCutFunction(cut_plane)
        cutter.SetGenerateTriangles(False)
        cutter.SetSortBy(1)
        cutter.SetNumberOfContours(1)
        cutter.SetInputConnection(clip.GetOutputPort())
        cutter.Update()

        if self.debug:
            cipped_mesh_file_path = "debug/clipped_mesh_segments/clipped_mesh_rotation_{0:03d}_segment_{1:03d}.vtp".format(r, i)
            saveVtp(os.path.join(DATA_PATH , cipped_mesh_file_path), clip.GetOutput())

            # if(i%2!=1):
            if(i == 52):
                self.debug_rotation_vector.addLine(centerline_segment_middle, rotated_cut_direction)

                plane_path = os.path.join(DATA_PATH , "debug/cut_plane_{0:03d}_segment_{1:03d}.vtp".format(r, i))
                plane_center = ((rotated_cut_direction - centerline_segment_middle) / 40 ) + centerline_segment_middle
                plane = pv.Plane(plane_center, cut_normal, 0.02, 0.05)
                # plane.rotate_z(45, true)
                plane.save(plane_path)
                saveVtp(os.path.join(os.path.join(DATA_PATH , "debug/cut_line_{0:03d}_segment_{1:03d}.vtp".format(r, i))), cutter.GetOutput())


        strip_one = vtk.vtkStripper()
        strip_one.SetInputData(cutter.GetOutput())
        strip_one.JoinContiguousSegmentsOn()
        strip_one.Update()

        decimateFilter = vtk.vtkDecimatePolylineFilter()
        decimateFilter.SetTargetReduction(0.001)
        decimateFilter.SetInputData(strip_one.GetOutput())
        decimateFilter.Update()

        return decimateFilter.GetOutput()

    def createRotationMatrix(self, point, old_point, normal):
        vx_norm = normalize(old_point - point)
        vy_norm = - normalize(np.cross(vx_norm, normal))
        vz_norm = - normalize(np.cross(vy_norm, vx_norm))

        R = np.asarray(np.column_stack((vx_norm, vy_norm, vz_norm)), dtype=np.float64)
        R = np.pad(R, ((0,1),(0,1)))
        R[3,3] = 1

        return R

    def createRotationMatrixUprightNormal(self, normal):
        vy_norm = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        vx_norm = normalize(np.cross(vy_norm, normal))
        vz_norm = - normalize(np.cross(vy_norm, vx_norm))

        R = np.asarray(np.column_stack((vx_norm, vy_norm, vz_norm)), dtype=np.float64)
        R = np.pad(R, ((0,1),(0,1)))
        R[3,3] = 1
        return R

    def createRotationMatrixAngledCenterLinePerpendicular(self, center_line_direction, center, angle):
        center[2] = 0
        center = - normalize(center)
        rot_axis = [-center[0], center[1], 0]

        vz_norm = normalize(rotatePointAroundAxis(center, rot_axis, -angle))
        vy_norm = normalize(np.cross(center_line_direction, vz_norm))
        vx_norm = - normalize(np.cross(vz_norm, vy_norm))

        R = np.asarray(np.column_stack((vx_norm, vy_norm, vz_norm)), dtype=np.float64)
        R = np.pad(R, ((0,1),(0,1)))
        R[3,3] = 1

        return R

    def createRotationMatrixUprightCenterLinePerpendicular(self, center):
        center[2] = 0

        vy_norm = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        vz_norm = - normalize(center)
        vx_norm = normalize(np.cross(vy_norm, vz_norm))

        R = np.asarray(np.column_stack((vx_norm, vy_norm, vz_norm)), dtype=np.float64)
        R = np.pad(R, ((0,1),(0,1)))
        R[3,3] = 1
        return R

    def createPose(self, point, rotation_matrix):
        q = tr.quaternion_from_matrix(rotation_matrix)
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation.w = q[3]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        return pose

    def createRotationSegmentRaster(self, combined_rotation_segment, clipped_mesh, toolpath_direction, rotation_degree, center, center_line_direction):
        points = combined_rotation_segment.GetPoints()

        normal_generator = vtk.vtkPolyDataNormals()
        normal_generator.SetInputData(clipped_mesh)
        normal_generator.ComputePointNormalsOn()
        normal_generator.ComputeCellNormalsOff()
        normal_generator.Update()
        mesh_segment_normals = normal_generator.GetOutput().GetPointData().GetArray("Normals")

        point_locator = vtk.vtkPointLocator()
        point_locator.SetDataSet(clipped_mesh)
        point_locator.AutomaticOn()
        point_locator.SetNumberOfPointsPerBucket(2)
        point_locator.BuildLocator

        rotation_matrix = np.asarray((4,4), dtype=np.float64)
        old_point = vtkPointToNumpyArray(points.GetPoint(0))
        raster = PoseArray()

        for i in range(1, points.GetNumberOfPoints()):
            point = vtkPointToNumpyArray(points.GetPoint(i))

            if euclideanDistancePose(point, old_point) > self.pose_spacing:

                point_id = point_locator.FindClosestPoint(point)
                normal = mesh_segment_normals.GetTuple(point_id)

                point_height = point[2]

                if(rotation_degree <= self.lower_rotation_end):
                   rotation_matrix  = self.createRotationMatrixUprightCenterLinePerpendicular(center)

                elif(rotation_degree >= self.lower_rotation_end and
                     point_height    <= self.lower_rotation_height):
                    continue

                else:
                    rotation_matrix = self.createRotationMatrixAngledCenterLinePerpendicular(center_line_direction, center, self.upper_spray_angle)

                raster.poses.append(self.createPose(point, rotation_matrix))

                if self.debug: self.debug_poses.addPose(point, rotation_matrix)

                old_point = copy.deepcopy(point)

        raster.poses.reverse() if toolpath_direction == "left" else None

        return raster

    def createToolpath(self, center_line_points, mesh_segments, clipped_mesh):
        toolpath_direction = "right"
        center = self.findCenterOfCoil(center_line_points)
        center_line_size = len(center_line_points)
        center_line_direction = normalize(- center_line_points[0] + center_line_points[-1])

        #fix random centerline direction
        rotation_direction = 1 if center_line_points[0][0] > center_line_points[center_line_size//2][0] else -1

        raster_degrees = []
        raster_array = []

        for r in range(self.rot_begin, self.rot_end + 1, self.rot_step):

            gap_filter = GapFilter(center_line_size)

            for i in range(0, center_line_size - 1): 

                axis = center_line_points[i + 1] - center_line_points[i] 

                if i == 0 or i == center_line_size -1: middle = center_line_points[0]
                else: middle = (center_line_points[i+1] - center_line_points[i])/2 + center_line_points[i]

                cut_direction = middle + normalize(np.cross(axis, center - middle))
                rotated_cut_direction = rotatePointAroundTranslatedAxis(center_line_points[i], axis, cut_direction, rotation_direction * (r - 90))
                cut_normal = np.cross(center_line_points[i+1] - rotated_cut_direction, center_line_points[i] - rotated_cut_direction)

                #cut surface to create toolpath segment
                rotation_segement = self.createRotationSegment(mesh_segments[i], cut_normal, rotated_cut_direction, middle, i, r)
                gap_filter.addSegment(rotation_segement)

                if(i == 25): 
                    saveDebugLine(center_line_points[i+1], center_line_points[i], os.path.join(DATA_PATH ,"debug/centerline_25.vtp"))
                    # egal = rotated_cut_direction - middle
                    # egal = [0, 0, 0.1]
                    # rectangle = pv.geometric_objects.Rectangle([center_line_points[i-1], center_line_points[i+1], center_line_points[i-1] + egal, center_line_points[i+1]] + egal)
                    # rectangle = pv.Line(center_line_points[i-1], center_line_points[i+1])
                    # rectangle.plot(show_edges=True, line_width=5)
                    # rectangle.save(os.path.join(DATA_PATH ,"debug/rectangle_" + str(r) + ".vtp"))

            combined_rotation_segment = gap_filter.getCombinedRotationSegement()

            if self.debug:
                self.debug_rotation_vector.saveVtp(DATA_PATH)

            raster = self.createRotationSegmentRaster(combined_rotation_segment, clipped_mesh, toolpath_direction, r, center, center_line_direction)
            raster_degrees.append(r)
            raster_array.append(raster)

            toolpath_direction = "right" if toolpath_direction == "left" else "left"
            gap_filter = None

        return raster_array, raster_degrees

    def computeCenterline(self, input_mesh_path, output_centerline_path):
        script_path = os.path.normpath( os.path.join(os.path.dirname(__file__), 'conda/toolpath_centerline.bash'))
        execute_string =  script_path + ' ' + input_mesh_path + ' ' + output_centerline_path
        rc = subprocess.call(execute_string, shell=True)

    def run(self, ply_path):
        start_time = timer()
        attempt = 0
        centerline_points = []

        while len(centerline_points) < self.centerline_minimal_point_size_treshold :
            # fill gaps and create mesh with open3d
            rospy.loginfo('Loading pointcload and closing gaps.')
            watertight_stl_path = os.path.join(DATA_PATH, 'tmp/watertight_coil.stl')
            cropAndFillGapsInMesh(ply_path, watertight_stl_path, self.z_clip_height, self.voxel_down_sample_size, self.debug)

            # smooth mesh
            watertight_mesh = loadStl(watertight_stl_path) 
            smoothed_mesh = smoothMesh(watertight_mesh, int(self.smoothing_mesh_factor))

            cleaner = vtk.vtkCleanPolyData()
            cleaner.SetInputData(smoothed_mesh)
            cleaner.Update()
            smoothed_clean_mesh = cleaner.GetOutput()

            # clip mesh
            clipped_mesh = clipMeshAtZaxis(smoothed_clean_mesh, self.z_clip_height)
            # if(self.debug): renderVtkPolydata(clipped_mesh)
            clipped_vtp_path = os.path.join(DATA_PATH, 'tmp/clipped_coil.vtp')
            saveVtp(clipped_vtp_path, clipped_mesh)
            
            # compute centerline
            centerline_path = os.path.join(DATA_PATH, 'tmp/centerline.vtp')
            self.computeCenterline(clipped_vtp_path, centerline_path)
            centerline_source = loadVtp(centerline_path)
            centerline = reducePolylinePointResolution(centerline_source, self.centerline_target_reduction)
            centerline_points = vtkPointsToNumpyArrayList(centerline.GetPoints())
            
            if len(centerline_points) < self.centerline_minimal_point_size_treshold:
                rospy.loginfo('Number of centerline points %i to small retrying!', len(centerline_points))
                attempt = attempt + 1     
            else:
                rospy.loginfo('Number of centerline points: %i', len(centerline_points))       

            if attempt > 10:
                rospy.logerr('Computing centerline failed after ten attempts!')
                return       

        if centerline_points[0][0] < centerline_points[0][1]: centerline_points.reverse()

        #split mesh in segments
        mesh_segments = self.splitMeshInSegments(centerline_points, clipped_mesh)

        #create toolpath poses
        raster_array, raster_degrees = self.createToolpath(centerline_points, mesh_segments, clipped_mesh)

        if self.debug:
            saveVtp(os.path.join(DATA_PATH, 'debug/centerline_reduced.vtp'), centerline)
            self.debug_poses.saveVtp(DATA_PATH)
            even_segments = vtk.vtkAppendPolyData()
            uneven_segments = vtk.vtkAppendPolyData()

            idx = 0
            for segment in mesh_segments:
                saveVtp(os.path.join(DATA_PATH , ('debug/mesh_segments/mesh_segment' + str(idx) + '.vtp')), segment)
                idx+=1
                even_segments.AddInputData(segment) if (idx%2 == 0) else uneven_segments.AddInputData(segment)
            
            even_segments.Update()
            uneven_segments.Update()
            saveVtp(os.path.join(DATA_PATH , ('debug/mesh_segments/even_segments.vtp')), even_segments.GetOutput())
            saveVtp(os.path.join(DATA_PATH , ('debug/mesh_segments/uneven_segments.vtp')), uneven_segments.GetOutput())


        marker_array = convertRasterArrayToAxisMarkers(raster_array, namespace="raster_array", frame_id=self.frame_id)
        self.publisher.publish(marker_array)

        number_of_poses = 0
        for pose_array in raster_array:
            number_of_poses += len(pose_array.poses)
        
        elapsed_time = timer() - start_time
        rospy.loginfo('Successfully createded toolpath with %i rasters and %i poses!', len(raster_array), number_of_poses)
        rospy.loginfo('Computation of toolpath took %i seconds', elapsed_time)
        return raster_array, raster_degrees

    def handle_request(self, req):
        response = GenerateTubularToolpathResponse()
        raster_array, raster_degrees = self.run(req.mesh_path)

        response.raster_degrees.data = raster_degrees
        response.raster_array = raster_array

        return response
