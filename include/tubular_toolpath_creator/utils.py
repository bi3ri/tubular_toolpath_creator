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

def reducePolylinePointResolution(polyline):
    decimateFilter = vtk.vtkDecimatePolylineFilter()
    decimateFilter.SetTargetReduction(0.97)
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

def euclideanDistancePose(p1, p2):
    x_dis = p1.position.x - p2.position.x
    y_dis = p1.position.y - p2.position.y
    z_dis = p1.position.z - p2.position.z
    return math.sqrt(x_dis**2 + y_dis**2 + z_dis**2)

def cropAndFillGapsInMesh(input_path, output_path, z_crop_height):
    pcd = o3d.io.read_point_cloud(input_path)
    crop_bounding_box = o3d.geometry.AxisAlignedBoundingBox([0,0, z_crop_height], [9999,9999,9999])
    cropped_pcd = pcd.crop(crop_bounding_box)
    # cropped_pcd = o3d.geometry.crop_point_cloud(pcd, (0,0,1), (0,0,0))
    print(cropped_pcd)
    # o3d.visualization.draw_geometries([cropped_pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = cropped_pcd.voxel_down_sample(voxel_size=0.01) #TODO: voxel_size as parameter
    # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([downpcd])
    pcd = downpcd
    print(downpcd)

    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(100)
    # o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
                mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=7) #higher depth -> higher detail
    print(mesh)
    # o3d.visualization.draw_geometries([mesh])

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
    # o3d.visualization.draw_geometries([density_mesh])


    # radii = [0.005, 0.01, 0.02, 0.04]
    # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([pcd, rec_mesh])

    mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
    o3d.io.write_triangle_mesh(output_path, mesh)
    # o3d.visualization.draw_geometries([mesh])


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



