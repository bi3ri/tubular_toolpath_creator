#!/usr/bin/env python
import vtk
import math
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt #fuer density
from scipy.spatial.transform import Rotation

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

def reducePolylinePointResolution(polyline):
    decimateFilter = vtk.vtkDecimatePolylineFilter()
    decimateFilter.SetTargetReduction(0.97)
    # decimateFilter.SetMaximumError(0.3)
    decimateFilter.SetInputData(polyline.GetOutput())
    decimateFilter.Update()
    return decimateFilter.GetOutput(0)

def smoothMesh(mesh, smoothing_mesh_factor):
    smooth = vtk.vtkSmoothPolyDataFilter()
    smooth.SetInputData(mesh)
    smooth.SetNumberOfIterations(smoothing_mesh_factor)
    # smooth.SetRelaxationFactor(0.6)
    smooth.Update()
    return smooth.GetOutput()

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

def directionVectorsToQuaternion(vx, vy, vz):
    R = np.array([vx, vy, vz])
    m = R

    qw = math.sqrt(1.0 + m[0,0] + m[1,1] + m[2,2]) / 2.0
    w4 = (4.0 * qw)
    qx = (m[2,1] - m[1,2] / w4)
    qy = (m[0,2] - m[2,0] / w4)
    qz = (m[1,0] - m[0,1] / w4)
    return qw, qx, qy, qz

def rotatePointAroundAxis(p, axis, theta):
    theta = math.radians(theta)
    axis = axis / np.linalg.norm(axis)  # normalize the rotation vector first
    rot = Rotation.from_rotvec(theta * axis)
    return rot.apply(p)

def fillGapsInMesh(input_path, output_path):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(input_path)
    print(pcd)
    print(np.asarray(pcd.points))

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.003)
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([downpcd])
    pcd = downpcd

    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(100)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=7) #higher depth -> higher detail
    print(mesh)
    o3d.visualization.draw_geometries([mesh])

    print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh])


    # radii = [0.005, 0.01, 0.02, 0.04]
    # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([pcd, rec_mesh])

    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    o3d.io.write_triangle_mesh(output_path, mesh)