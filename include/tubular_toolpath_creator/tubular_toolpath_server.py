#!/usr/bin/env python
import rospy

import pyvista as pv
import vtk
import numpy as np
from geometry_msgs.msg import Pose, PoseArray


from tubular_toolpath_creator.gap_filter import GapFilter

# from gap_filter import GapFilter
from tubular_toolpath_creator.utils import loadStl, loadVtp, polyDataToActor, lines_from_points, normalize, lookAt, directionVectorsToQuaternion, reducePolylinePointResolution, rotatePointAroundAxis, fillGapsInMesh, euclideanDistancePose
from tubular_toolpath_creator.srv import GenerateTubularToolpath

class TubularToolpathServer:
    nodeHandle = None
    plotter = pv.Plotter()
    smoothing_mesh_factor = 5000
    pose_spacing = 100
    # colors = vtk.vtkNamedColors() 
    rot_begin = 10
    rot_end = 210
    rot_step = 40

    def __init__(self):
        self.rot_begin = rospy.get_param('~tubular_toolpath_creator', 'rotation_begin') #hier parameter von nodeHandle
        self.rot_end = rospy.get_param('~tubular_toolpath_creator', 'rotation_end')
        self.rot_step = rospy.get_param('~tubular_toolpath_creator', 'rotation_step')

        service = rospy.Service('create_tubular_toolpath', GenerateTubularToolpath, self.handle_request)

    # def reducePolylinePointResolution(center_line_source):
    #     decimateFilter = vtk.vtkDecimatePolylineFilter()
    #     decimateFilter.SetTargetReduction(0.97)
    #     # decimateFilter.SetMaximumError(0.3)
    #     decimateFilter.SetInputData(center_line_source.GetOutput())
    #     decimateFilter.Update()
    #     center_line_source_reduced = decimateFilter.GetOutput(0)
    #     center_line = pv.wrap(center_line_source_reduced)
    #     center_line_points = center_line.points
    #     return center_line_points

    def findCenterOfCoil(self, center_line_points):
        center = np.mean(center_line_points, axis=0)
        center[2] = center_line_points[0][2] 
        return center

    # def smoothMesh(self, coil_mesh_source):
    #     smooth = vtk.vtkSmoothPolyDataFilter()
    #     smooth.SetInputData(coil_mesh_source)
    #     smooth.SetNumberOfIterations(self.smoothing_mesh_factor)
    #     # smooth.SetRelaxationFactor(0.6)
    #     smooth.Update()
    #     smoothed_mesh = smooth.GetOutput()
    #     return smoothed_mesh

    def splitMeshInSegments(self, center_line_points, mesh):
        center_line_size = center_line_points.shape[0]
        middle = center_line_size // 2
        clipped_coil_array = [None] * (center_line_size - 1)

        clip_plane_middle = vtk.vtkPlane()
        normal = center_line_points[middle] - center_line_points[middle+1]
        clip_plane_middle.SetNormal(normal)
        clip_plane_middle.SetOrigin(center_line_points[middle]) 

        clipper_middle = vtk.vtkClipPolyData()
        clipper_middle.SetGenerateClippedOutput(True)
        clipper_middle.SetInputData(mesh)
        clipper_middle.SetClipFunction(clip_plane_middle)
        clipper_middle.Update()

        right_mesh = clipper_middle.GetOutput(0)
        left_mesh = clipper_middle.GetOutput(1)

        remaining_mesh = left_mesh
        for i in range(middle, center_line_size - 1):
            clip = vtk.vtkClipPolyData()
            clip.SetGenerateClippedOutput(True)

            clip_plane = vtk.vtkPlane()
            normal = center_line_points[i+1] - center_line_points[i]
            clip_plane.SetNormal(normal)
            clip_plane.SetOrigin(center_line_points[i+1]) 
            clip.SetInputData(remaining_mesh) 
            clip.SetClipFunction(clip_plane)
            clip.Update()

            clipped_coil_array[i] = clip.GetOutput(1)
            remaining_mesh = clip.GetOutput(0)

        remaining_mesh = right_mesh
        for i in range(0, middle):
            clip = vtk.vtkClipPolyData()
            clip.SetGenerateClippedOutput(True)

            clip_plane = vtk.vtkPlane()
            normal = center_line_points[i+1] - center_line_points[i]
            clip_plane.SetNormal(normal)
            clip_plane.SetOrigin(center_line_points[i+1]) 
            clip.SetInputData(remaining_mesh) 
            clip.SetClipFunction(clip_plane)
            clip.Update()

            clipped_coil_array[i] = clip.GetOutput(1)
            remaining_mesh = clip.GetOutput(0)

        return clipped_coil_array


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
        #
        cut_plane = vtk.vtkPlane()
        cut_normal = np.cross(cut_normal)
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
        
        #TODO: not sure if i need the next two blocks
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
        combined_rotation_segment_pose_array = PoseArray()

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


        #start pose
        pose = Pose()
        next_pose = Pose()
        next_pose.position.x = combined_rotation_segment_points.GetPoint(first_id)[0]
        next_pose.position.y = combined_rotation_segment_points.GetPoint(first_id)[1]
        next_pose.position.z = combined_rotation_segment_points.GetPoint(first_id)[2]
            
        for i in range(start_id , end_id):
            
            pose = next_pose
            next_pose.position.x = combined_rotation_segment_points.GetPoint(i)[0]
            next_pose.position.y = combined_rotation_segment_points.GetPoint(i)[1]
            next_pose.position.z = combined_rotation_segment_points.GetPoint(i)[2]

            if euclideanDistancePose(pose, next_pose) < self.pose_spacing:
                continue

            #get vz direction vector 
            vz_norm = normalize(rot_center - [pose.position.x, pose.position.y, pose.position.z])

            toolpath_direction_vector = (np.array([next_pose.position.x, next_pose.position.y, next_pose.position.z])
                                        - np.array([pose.position.x, pose.position.y, pose.position.z]))
            vy_norm = normalize(np.cross(vz_norm, toolpath_direction_vector))

            vx_norm = normalize(np.cross(vz_norm, vy_norm))

            qw, qx, qy, qz = directionVectorsToQuaternion(vx_norm, vy_norm, vz_norm)

            pose.orientation.w = qw
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz

            combined_rotation_segment_pose_array.poses.append(pose)

        #last pose gets orientation from point before rotation
        last_pose = Pose()
        last_pose.position.x = combined_rotation_segment_points.GetPoint(end_id)[0]
        last_pose.position.y = combined_rotation_segment_points.GetPoint(end_id)[1]
        last_pose.position.z = combined_rotation_segment_points.GetPoint(end_id)[2]
        last_pose.orientation.w = pose.orientation.w
        last_pose.orientation.x = pose.orientation.x
        last_pose.orientation.y = pose.orientation.y
        last_pose.orientation.z = pose.orientation.z

        combined_rotation_segment_pose_array.poses.append(last_pose)

        return combined_rotation_segment_pose_array, toolpath_direction

    def createToolpath(self, center_line_points, mesh_segments):
        toolpath_raster = []
        toolpath_direction = "right"

        center = self.findCenterOfCoil(center_line_points)
        center_line_size = center_line_points.shape[0]

        gap_filter = GapFilter(center_line_size)

        for r in range (self.rot_begin, self.rot_end, self.rot_step):
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

                #closing gaps between two cut plane segments
                gap_filter.addSegment(rotation_segement, center_line_size, i)

            gap_filter.addLastSegment(rotation_segement)
            combined_rotation_segment = gap_filter.getCombinedRotationSegement()

            ### create toolpath poses
            combined_rotation_segment_pose_array, toolpath_direction = self.createRotationSegmentPoseArray(combined_rotation_segment, toolpath_direction, rot_center)
            toolpath_raster.append(combined_rotation_segment_pose_array)

        return toolpath_raster


    def run(self, mesh_path, centerline_path):
        fillGapsInMesh(mesh_path, mesh_path + '_watertight')
        mesh = loadStl(mesh_path + '_watertight')
        smoothed_mesh = self.smoothMesh(mesh)

        centerline_source = loadVtp(centerline_path)
        centerline = reducePolylinePointResolution(centerline_source)
        centerline_points = (pv.wrap(centerline)).points
        # centerline_size = centerline_points.shape[0]
        
        mesh_segments = self.splitMeshInSegments(centerline_points, smoothed_mesh)

        toolpath_raster = self.createToolpath(centerline_points, mesh_segments)

        return toolpath_raster

    def handle_request(self, req):
        toolpath_raster = self.run(req.mesh_path, req.centerline_path)

        response = GenerateTubularToolpath()
        response.res = toolpath_raster

        return(response)


