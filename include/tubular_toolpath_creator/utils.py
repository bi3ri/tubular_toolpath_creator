#!/usr/bin/env python
import vtk
import math
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt #fuer density
from scipy.spatial.transform import Rotation
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def loadStl(fname):
    """Load the given STL file, and return a vtkPolyData object."""
    reader = vtk.vtkSTLReader()
    reader.SetFileName(fname)
    reader.Update()
    return reader.GetOutput()

def loadVtp(path):
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(path)
    reader.Update()
    return reader.GetOutput()

def loadPly(path):
    reader = vtk.vtkPLYReader()
    reader.SetFileName(path)
    reader.Update()
    return reader.GetOutput()
    
def savePly(path, mesh):
    writer = vtk.vtkPLYWriter()
    writer.SetFileName(path)
    # writer.vtkSetFilePathMacro(path)
    writer.SetInputData(mesh)
    # writer.SetFileTypeToASCII()
    # writer.Update()
    writer.Write()

def saveVtp(path, mesh):
    writer = vtk.vtkXMLPolyDataWriter()
    writer.SetFileName(path)
    # writer.vtkSetFilePathMacro(path)
    writer.SetInputData(mesh)
    # writer.Update()
    writer.Write()

def reducePolylinePointResolution(polyline, targetReduction):
    decimateFilter = vtk.vtkDecimatePolylineFilter()
    decimateFilter.SetTargetReduction(targetReduction)
    # decimateFilter.SetMaximumError(0.3)
    decimateFilter.SetInputData(polyline)
    decimateFilter.Update()
    return decimateFilter.GetOutput(0)

def smoothMesh(mesh, smoothing_mesh_factor):
    smooth = vtk.vtkSmoothPolyDataFilter()
    smooth.SetInputData(mesh)
    smooth.SetNumberOfIterations(smoothing_mesh_factor)
    # smooth.SetRelaxationFactor(0.6)
    smooth.Update()
    return smooth.GetOutput()

def clipMeshAtZaxis(mesh, z_height):
    clip_plane_middle = vtk.vtkPlane()
    clip_plane_middle.SetNormal((0,0,1))
    clip_plane_middle.SetOrigin((0,0, z_height)) 

    clipper = vtk.vtkClipPolyData()
    clipper.SetGenerateClippedOutput(True)
    clipper.SetInputData(mesh)
    clipper.SetClipFunction(clip_plane_middle)
    clipper.Update()

    return clipper.GetOutput()

def polyDataToActor(polydata):
    """Wrap the provided vtkPolyData object in a mapper and an actor, returning
    the actor."""
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        #mapper.SetInput(reader.GetOutput())
        mapper.SetInput(polydata)
    else:
        mapper.SetInputConnection(polydata.GetProducerPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    #actor.GetProperty().SetRepresentationToWireframe()
    actor.GetProperty().SetColor(0.5, 0.5, 1.0)
    return actor

def lines_from_points(points):
    """Given an array of points, make a line set"""
    poly = pv.PolyData()
    poly.points = points
    cells = np.full((len(points)-1, 3), 2, dtype=np.int_)
    cells[:, 1] = np.arange(0, len(points)-1, dtype=np.int_)
    cells[:, 2] = np.arange(1, len(points), dtype=np.int_)
    poly.lines = cells
    return poly

def normalize(v):
    return np.true_divide(v, np.linalg.norm(v))

def lookAt(eye, target):
    mz = normalize( (eye[0]-target[0], eye[1]-target[1], eye[2]-target[2]) ) # inverse line of sight
    mx = normalize( np.cross( [0,0,1], mz ) )
    my = normalize( np.cross( mz, mx ) )
    tx =  np.dot( mx, eye )
    ty =  np.dot( my, eye )
    tz = -np.dot( mz, eye )   
    return np.array([mx[0], my[0], mz[0], 0, mx[1], my[1], mz[1], 0, mx[2], my[2], mz[2], 0, tx, ty, tz, 1])

def orthogonalizeMatrix(A):
    r = (np.transpose(A) - A) / (1 + np.trace(A))
    cay_inner = np.identity(3) + r
    return np.linalg.inv(cay_inner) * cay_inner

def directionVectorsToQuaternion(vx, vy, vz):
    rotation_matrix = np.array([vx, vy, vz])
    r = Rotation.from_matrix(rotation_matrix)
    return r.as_quat()

# def directionVectorsToQuaternion(vx, vy, vz):
#     m = np.array([vx, vy, vz])
#     quat = tf.transformations.quaternion_from_matrix(m)
#     return quat

def rotatePointAroundAxis(p, axis, theta):
    theta = math.radians(theta)
    axis = axis / np.linalg.norm(axis)  # normalize the rotation vector first
    rot = Rotation.from_rotvec(theta * axis)
    return rot.apply(p)

def euclideanDistancePose(p1, p2):
    x_dis = p1[0] - p2[0]
    y_dis = p1[1] - p2[1]
    z_dis = p1[2] - p2[2]
    return math.sqrt(x_dis**2 + y_dis**2 + z_dis**2)

def cropAndFillGapsInMesh(input_path, output_path, z_crop_height, voxel_down_sample_size = 0.01, debug=False):
    pcd = o3d.io.read_point_cloud(input_path)
    crop_bounding_box = o3d.geometry.AxisAlignedBoundingBox([0,0, z_crop_height], [9999,9999,9999])
    cropped_pcd = pcd.crop(crop_bounding_box)
    if debug: o3d.visualization.draw_geometries([cropped_pcd])

    down_pcd = cropped_pcd.voxel_down_sample(voxel_down_sample_size) 
    if debug: o3d.visualization.draw_geometries([down_pcd])
    
    down_pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals
    down_pcd.estimate_normals()
    down_pcd.orient_normals_consistent_tangent_plane(100)
    if debug: o3d.visualization.draw_geometries([down_pcd], point_show_normal=True)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Info) as cm:
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(down_pcd, depth=7) #higher depth -> higher detail
    if debug: o3d.visualization.draw_geometries([mesh])

    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    if debug:  o3d.visualization.draw_geometries([density_mesh])

    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    if debug: o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh(output_path, mesh)


def renderVtkPolydata(polydata):
    """Wrap the provided vtkPolyData object in a mapper and an actor, returning
    the actor."""
    mapper = vtk.vtkPolyDataMapper()
    # if vtk.VTK_MAJOR_VERSION <= 5:
    #     #mapper.SetInput(reader.GetOutput())
    #     mapper.SetInput(polydata)
    # else:
    #     mapper.SetInputConnection(polydata.GetProducerPort())
    mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    #actor.GetProperty().SetRepresentationToWireframe()
    actor.GetProperty().SetColor(0.5, 0.5, 1.0)

    colors = vtk.vtkNamedColors()   

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    renderer.AddViewProp(actor)
    renderer.SetBackground(colors.GetColor3d('White'))
    renderer.GetActiveCamera().Pitch(90)
    renderer.GetActiveCamera().SetViewUp(0, 0, 1)
    renderer.ResetCamera()

    renderWindow.SetSize(2000, 2000)
    renderWindow.Render()
    renderWindow.SetWindowName('ReadPolyData')
    renderWindowInteractor.Start()

def convertPoseArrayToAxisMarkers(pose_arrays, 
                        namespace = 'debug_poses', 
                        frame_id = 'world', 
                        offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                        axis_scale = 0.001, 
                        axis_length = 0.01):

    marker_array = MarkerArray()
    red = [1.0, 0.0, 0.0, 1.0]
    green = [0.0, 1.0, 0.0, 1.0]
    blue = [0.0, 0.0, 1.0, 1.0]

    x_axis_marker = _create_line_marker(red, frame_id, namespace, axis_scale)
    y_axis_marker = _create_line_marker(green, frame_id, namespace, axis_scale)
    z_axis_marker = _create_line_marker(blue, frame_id, namespace, axis_scale)

    x_direction = [1, 0, 0]
    y_direction = [0, 1, 0]
    z_direction = [0, 0, 1]

    for pose_array in pose_arrays:
        for pose in pose_array.poses:

            p1, p2 = _create_axis_points(pose, x_direction, axis_length)
            x_axis_marker.points.append(p1)
            x_axis_marker.points.append(p2)
            x_axis_marker.pose.position.x = 0.0
            x_axis_marker.pose.position.y = 0.0
            x_axis_marker.pose.position.z = 0.0
            x_axis_marker.pose.orientation.x = 0.0
            x_axis_marker.pose.orientation.y = 0.0
            x_axis_marker.pose.orientation.z = 0.0
            x_axis_marker.pose.orientation.w = 1.0

            p1, p2 = _create_axis_points(pose, y_direction, axis_length)
            y_axis_marker.points.append(p1)
            y_axis_marker.points.append(p2)
            y_axis_marker.pose.position.x = 0.0
            y_axis_marker.pose.position.y = 0.0
            y_axis_marker.pose.position.z = 0.0
            y_axis_marker.pose.orientation.x = 0.0
            y_axis_marker.pose.orientation.y = 0.0
            y_axis_marker.pose.orientation.z = 0.0
            y_axis_marker.pose.orientation.w = 1.0

            p1, p2 = _create_axis_points(pose, z_direction, axis_length)
            z_axis_marker.points.append(p1)
            z_axis_marker.points.append(p2)
            z_axis_marker.pose.position.x = 0.0
            z_axis_marker.pose.position.y = 0.0
            z_axis_marker.pose.position.z = 0.0
            z_axis_marker.pose.orientation.x = 0.0
            z_axis_marker.pose.orientation.y = 0.0
            z_axis_marker.pose.orientation.z = 0.0
            z_axis_marker.pose.orientation.w = 1.0

    marker_array.markers.append(x_axis_marker)
    marker_array.markers.append(y_axis_marker)
    marker_array.markers.append(z_axis_marker)

    id = 0
    for marker in marker_array.markers:
        marker.id = id
        id += 1

    return marker_array

def convertDirectionVectorPoseToAxisMarkers(pose_array, 
                        namespace = 'debug_poses', 
                        frame_id = 'tsdf_origin', 
                        offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                        axis_scale = 0.001, 
                        axis_length = 0.01):
    
    marker_array = MarkerArray()
    red = [1.0, 0.0, 0.0, 1.0]
    green = [0.0, 1.0, 0.0, 1.0]
    blue = [0.0, 0.0, 1.0, 1.0]

    x_axis_marker = _create_line_marker(red, frame_id, namespace, axis_scale)
    y_axis_marker = _create_line_marker(green, frame_id, namespace, axis_scale)
    z_axis_marker = _create_line_marker(blue, frame_id, namespace, axis_scale)

    for i in range(len(pose_array)//12):
        p = Point()
        vx = Point()
        vy = Point()
        vz = Point()

        idx = i * 12

        p.x = pose_array[idx+0]
        p.y = pose_array[idx+1]
        p.z = pose_array[idx+2]
        vx.x = (pose_array[idx+3] * axis_length) + p.x
        vx.y = (pose_array[idx+4] * axis_length) + p.y
        vx.z = (pose_array[idx+5] * axis_length) + p.z
        vy.x = (pose_array[idx+6] * axis_length) + p.x
        vy.y = (pose_array[idx+7] * axis_length) + p.y
        vy.z = (pose_array[idx+8] * axis_length) + p.z
        vz.x = (pose_array[idx+9] * axis_length) + p.x
        vz.y = (pose_array[idx+10] * axis_length) + p.y
        vz.z = (pose_array[idx+11] * axis_length) + p.z

        x_axis_marker.points.append(p)
        x_axis_marker.points.append(vx)

        y_axis_marker.points.append(p)
        y_axis_marker.points.append(vy)

        z_axis_marker.points.append(p)
        z_axis_marker.points.append(vz)

    marker_array.markers.append(x_axis_marker)
    marker_array.markers.append(y_axis_marker)
    marker_array.markers.append(z_axis_marker)

    id = 0
    for marker in marker_array.markers:
        marker.id = id
        id += 1

    return marker_array

def _create_line_marker(color, frame_id, namespace, axis_scale):
    line_marker = Marker()
    line_marker.type = line_marker.ADD
    line_marker.color.r = float(color[0])
    line_marker.color.g = float(color[1])
    line_marker.color.b = float(color[2])
    line_marker.color.a = float(color[3])
    line_marker.header.frame_id = frame_id
    # line_marker.header.stamp = rospy.Time.now()
    line_marker.type = line_marker.LINE_LIST
    line_marker.lifetime = rospy.Duration(0)
    line_marker.ns = namespace
    # line_marker.scale = (axis_scale, 0, 0)
    line_marker.scale.x = axis_scale
    line_marker.scale.y = 0.0
    line_marker.scale.z = 0.0
    return line_marker

def _create_axis_points(pose, direction, axis_length):
    point1 = Point()
    point1.x = float(pose.position.x)
    point1.y = float(pose.position.y)
    point1.z = float(pose.position.z)

    # qx, qy, qz, qw =  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    r = Rotation.from_quat(quat)

    point2_coordinates = (r.apply(direction)) #+ [point1.x, point1.y, point1.z]
    point2 = Point()
    point2.x = float((point2_coordinates[0]* axis_length) + point1.x) 
    point2.y = float((point2_coordinates[1]* axis_length) + point1.y)
    point2.z = float((point2_coordinates[2]* axis_length) + point1.z)
    
    return point1, point2

