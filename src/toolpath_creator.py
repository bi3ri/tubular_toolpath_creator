import pyvista as pv
import vtk
import numpy as np
import math
from scipy.spatial.transform import Rotation

from pyvista import examples
#https://www.brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html


import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseArray


smoothing_mesh_factor = 5000



debug_lines = vtk.vtkAppendPolyData()



#list of geometry msgs
hotsprayToolpathMSG = []









def loadStl(fname):
    """Load the given STL file, and return a vtkPolyData object for it."""
    reader = vtk.vtkSTLReader()
    reader.SetFileName(fname)
    reader.Update()
    polydata = reader.GetOutput()
    return polydata

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

def rotatePointAroundAxis(p, axis, theta):
    # axis = np.asarray(axis)
    # axis = axis / math.sqrt(np.dot(axis, axis))
    # a = math.cos(theta / 2.0)
    # b, c, d = -axis * math.sin(theta / 2.0)
    # aa, bb, cc, dd = a * a, b * b, c * c, d * d
    # bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    # R = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
    #                  [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
    #                  [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    theta = math.radians(theta)
    axis = axis / np.linalg.norm(axis)  # normalize the rotation vector first
    rot = Rotation.from_rotvec(theta * axis)
    return rot.apply(p)







plotter = pv.Plotter()
colors = vtk.vtkNamedColors()   

filename = 'coil_full.ply'

coil_ply = vtk.vtkPLYReader()
coil_ply.SetFileName(filename)
coil_ply.Update()


filename = 'Coil2_centerline.vtp'
center_line_source = vtk.vtkXMLPolyDataReader()
center_line_source.SetFileName(filename)
center_line_source.Update()
coil_mesh_source = loadStl('coil2_capped.stl')

# scale = (1000,1000,1000)
# transform = vtk.vtkTransform()
# transform.Scale(scale)
# transformFilter = vtk.vtkTransformPolyDataFilter()
# transformFilter.SetInputData(coil_mesh_source)
# transformFilter.SetTransform(transform)
# transformFilter.Update()
# coil_mesh_scaled = transformFilter.GetOutputPort()





# reduce polyline point resolution 
decimateFilter = vtk.vtkDecimatePolylineFilter()
decimateFilter.SetTargetReduction(0.97)
# decimateFilter.SetMaximumError(0.3)
decimateFilter.SetInputData(center_line_source.GetOutput())
decimateFilter.Update()
center_line_source_reduced = decimateFilter.GetOutput(0)
center_line = pv.wrap(center_line_source_reduced)

center_line_points = center_line.points
center = np.mean(center_line_points, axis=0)
center[2] = center_line_points[0][2] #z wird auf z-Wert von erstem Punkt in der centerline gesetzt






# pv_reduced_line = pv.wrap(center_line_points)
# # center_line.plot()

# planeSource = vtk.vtkPlaneSource()
# planeSource.SetOrigin(center_line_points[1])

# planeSource.SetPoint1(center_line_points[0])
# planeSource.SetPoint2(mean)
# # planeSource.SetXResolution(20)
# # planeSource.SetYResolution(20)
# planeSource.Update()
# plane = planeSource.GetOutput()

# cleanFilter = vtk.vtkCleanPolyData()
# cleanFilter.SetInputData(coil_mesh_source)
# cleanFilter.SetAbsoluteTolerance(0.01)
# cleanFilter.Update()

# fill = vtk.vtkFillHolesFilter()
# fill.SetInputConnection(cleanFilter.GetOutputPort())   
# fill.SetHoleSize(100)    
# fill.Update()



smooth = vtk.vtkSmoothPolyDataFilter()
smooth.SetInputData(coil_mesh_source)
smooth.SetNumberOfIterations(smoothing_mesh_factor)
# smooth.SetRelaxationFactor(0.6)
smooth.Update()
smoothed_mesh = smooth.GetOutput()







# Seperate Mesh in Segments

center_line_size = center_line_points.shape[0]
middle = center_line_size // 2
clipped_coil_array = [None] * (center_line_size - 1)

clip_plane_middle = vtk.vtkPlane()
normal = center_line_points[middle] - center_line_points[middle+1]
clip_plane_middle.SetNormal(normal)
clip_plane_middle.SetOrigin(center_line_points[middle]) 

clipper_middle = vtk.vtkClipPolyData()
clipper_middle.SetGenerateClippedOutput(True)
clipper_middle.SetInputData(smoothed_mesh)
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





polyline = vtk.vtkPolyData()

rot_begin = 10
rot_end = 210
rot_step = 40

toolpath_raster = []
pre_combined_rotation_segment = vtk.vtkAppendPolyData()
start_pt = None
end_pt = None
start_pt_id = 0
end_pt_id = 0
old_end_pt = None
old_end_id = 0
output = None
old_rotation_segement = None
toolpath_direction = "right"

for r in range (rot_begin, rot_end, rot_step):
    for i in range(0, center_line_size - 1):
        axis = center_line_points[i + 1] - center_line_points[i] 
        #axis_norm = axis / np.linalg.norm(axis)

        #translate roation_center relative to coordinate ursprung, rotate and translate back
        translation_center = center - center_line_points[i] 
        rot_center = rotatePointAroundAxis(translation_center, axis, r)
        rot_center += center_line_points[i]

        middle = (center_line_points[i+1] - center_line_points[i])/2 + center_line_points[i]


        #cut front of tubular surface relative to rot_center in order to not cut surface twice
        clip = vtk.vtkClipPolyData()
        # clip.SetGenerateClippedOutput(True)
        clip_plane = vtk.vtkPlane()
        clip_plane.SetNormal(rot_center - middle)
        clip_plane.SetOrigin(middle) 
        clip.SetInputData(clipped_coil_array[i]) 
        clip.SetClipFunction(clip_plane)
        clip.Update()
        #

        #cut surface to create toolpath segment
        cut_plane = vtk.vtkPlane()
        cut_normal = np.cross(center_line_points[i+1] - rot_center, center_line_points[i] - rot_center)
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
        
        
        #not sure if i need the next two blocks
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


        #closing gaps between two cut plane segments
        number_of_cells = rotation_segement.GetLines().GetNumberOfConnectivityIds()
        p1_id = rotation_segement.GetLines().GetData().GetValue(number_of_cells)
        p2_id = rotation_segement.GetCell(0).GetPointId(0)
        p1 = rotation_segement.GetPoint(p1_id)
        p2 = rotation_segement.GetPoint(p2_id)

        # A  
        if i == 0:
            #check which point is lower on z axis
            if p1[2] < p2[2]:
                old_end_pt = p2
                old_end_id = p2_id
            else:
                old_end_pt = p1
                old_end_id = p1_id
            old_rotation_segement = rotation_segement

        #B
        if i > 0 & i < center_line_size:
            p1_distance = vtk.vtkMath.Distance2BetweenPoints(old_end_pt, p1)
            p2_distance = vtk.vtkMath.Distance2BetweenPoints(old_end_pt, p2)

            if p1_distance < p2_distance:
                start_pt = p1
                end_pt = p2
                start_pt_id = p1_id
                end_pt_id = p2_id
            else:
                start_pt = p2
                end_pt = p1
                start_pt_id = p2_id
                end_pt_id = p1_id

            mx = ((old_end_pt[0] - start_pt[0]) / 2) + start_pt[0]
            my = ((old_end_pt[1] - start_pt[1]) / 2) + start_pt[1]
            mz = ((old_end_pt[2] - start_pt[2]) / 2) + start_pt[2]
            m = (mx, my, mz)

            rotation_segement.GetPoints().SetPoint(start_pt_id, m)
            old_rotation_segement.GetPoints().SetPoint(old_end_id, m)

            pre_combined_rotation_segment.AddInputData(old_rotation_segement)
            old_rotation_segement = rotation_segement
            old_end_id = end_pt_id
            old_end_pt = end_pt


    #add last rotation_segement to pre_combined_rotation_segment
    pre_combined_rotation_segment.AddInputData(rotation_segement)
    pre_combined_rotation_segment.Update()

    #create one polyline out of many lines
    strip = vtk.vtkStripper()
    strip.SetInputData(pre_combined_rotation_segment.GetOutput())
    strip.JoinContiguousSegmentsOn()
    # strip.SetMaximumLength(1)
    strip.Update()
    combined_rotation_segment = strip.GetOutput()
    toolpath_raster.append(combined_rotation_segment)

    ### create toolpath poses

    combined_rotation_segment_points = combined_rotation_segment.GetPoints()
    
    combined_rotation_segment_pose_array = PoseArray()


    if toolpath_direction == "right":
        start_id = 1
        end_id = combined_rotation_segment_points.GetNumberOfPoints() - 1
        first_id = 0
    else:
        start_id = combined_rotation_segment_points.GetNumberOfPoints() - 1
        end_id = 0
        first_id = start_id - 1

    #start pose
    pose = Pose()
    next_pose = Pose()
    next_pose.position.x = combined_rotation_segment_points.GetPoint(start_id)[0]
    next_pose.position.y = combined_rotation_segment_points.GetPoint(start_id)[1]
    next_pose.position.z = combined_rotation_segment_points.GetPoint(start_id)[2]
        

    for i in range(start_id , end_id):
        pose = next_pose
        next_pose.position.x = combined_rotation_segment_points.GetPoint(i)[0]
        next_pose.position.y = combined_rotation_segment_points.GetPoint(i)[1]
        next_pose.position.z = combined_rotation_segment_points.GetPoint(i)[2]

        #get vz direction vector 
        # minus mitte + center
        vz = (rot_center - [pose.position.x, pose.position.y, pose.position.z])
        vz_norm = vz / np.linalg.norm(vz)

        toolpath_direction_vector = (np.array([next_pose.position.x, next_pose.position.y, next_pose.position.z])
                                    - np.array([pose.position.x, pose.position.y, pose.position.z]))
        vy = np.cross(vz, toolpath_direction_vector)
        vy_norm = vy / np.linalg.norm(vy)

        vx = np.cross(vz, vy)
        vx_norm = vx / np.linalg.norm(vx)

        R = np.array([vx_norm, vy_norm, vz_norm])
        m = R

        qw = math.sqrt(1.0 + m[0,0] + m[1,1] + m[2,2]) / 2.0
        w4 = (4.0 * qw)
        qx = (m[2,1] - m[1,2] / w4)
        qy = (m[0,2] - m[2,0] / w4)
        qz = (m[1,0] - m[0,1] / w4)

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

    hotsprayToolpathMSG.append(combined_rotation_segment_pose_array)

    
















   
debug_line = vtk.vtkLineSource()
debug_line.SetPoint1((0, 0,  0 ))
debug_line.SetPoint2(smoothed_mesh.GetPoint(3))
debug_line.Update()
debug_lines.AddInputData(debug_line.GetOutput())

normal_filter = vtk.vtkPolyDataNormals()
normal_filter.ComputeCellNormalsOn()
normal_filter.ComputePointNormalsOff()
normal_filter.SetInputData(smoothed_mesh)
#normal_filter.UpdateInformation()
normal_filter.SetFeatureAngle(25)
normal_filter.Update()
normal_array = normal_filter.GetOutput().GetCellData().GetNormals()



#SUCHE NACH FUNKTIN IN WELCHEM DREIECK LIEGT DER PUNKT DER GERADE
#DANN MEAN NORMALE AUSRECHNEN

#create roboter poeses

# for i in range(0, segments.count):
# for segment in segments:

#     for i in range(0, segment.GetPoints().GetNumberOfPoints()):

#         debug_line = vtk.vtkLineSource()
#         debug_line.SetPoint1((0,0,0))
#         debug_line.SetPoint2(segment.GetPoint(3))
#         debug_line.Update()
#         debug_lines.AddInputData(debug_line.GetOutput())




#         break
#         idx = 0
#         vz = (0, 0, 0)
#         segment.GetPointData().GetNormals().GetTuple(idx, vz.data())



clipper_mapper = vtk.vtkPolyDataMapper()
clipper_mapper.SetInputData(clip.GetOutput())
clipper_actor = vtk.vtkActor()
# clipper_actor.GetProperty().SetOpacity(.5)
clipper_actor.SetMapper(clipper_mapper)
clipper_actor.GetProperty().SetColor(colors.GetColor3d('Blue'))

mesh_mapper = vtk.vtkPolyDataMapper()
mesh_mapper.SetInputData(left_mesh)
mesh_actor = vtk.vtkActor()
mesh_actor.GetProperty().SetOpacity(.3)
mesh_actor.SetMapper(mesh_mapper)
mesh_actor.GetProperty().SetColor(colors.GetColor3d('Yellow'))

# intersect_mapper = vtk.vtkPolyDataMapper()
# intersect_mapper.SetInputData(combined_rotation_segment.GetOutput())
# intersect_actor = vtk.vtkActor()
# intersect_actor.SetMapper(intersect_mapper)
# intersect_actor.GetProperty().SetColor(colors.GetColor3d('Yellow'))

plane_mapper = vtk.vtkPolyDataMapper()
# plane_mapper.SetInputConnection(clip_plane)
plane_actor = vtk.vtkActor()
plane_actor.GetProperty().SetOpacity(.5)
plane_actor.SetMapper(plane_mapper)
plane_actor.GetProperty().SetColor(colors.GetColor3d('Orange'))

line_mapper = vtk.vtkPolyDataMapper()
line_mapper.SetInputData(center_line)
line_actor = vtk.vtkActor()
line_actor.SetMapper(line_mapper)
line_actor.GetProperty().SetColor(colors.GetColor3d('Red'))

debug_lines.Update()
debug_line_mapper = vtk.vtkPolyDataMapper()
debug_line_mapper.SetInputData(debug_lines.GetOutput())
debug_line_actor = vtk.vtkActor()
debug_line_actor.SetMapper(debug_line_mapper)
debug_line_actor.GetProperty().SetColor(colors.GetColor3d('Red'))

# point_mapper = vtk.vtkPolyDataMapper()
# point_mapper.SetInputData(append_filling.GetOutput())
# point_actor = vtk.vtkActor()
# point_actor.SetMapper(point_mapper)
# point_actor.GetProperty().SetColor(colors.GetColor3d('Red'))

# cut_line_mapper = vtk.vtkPolyDataMapper()
# cut_line_mapper.SetInputData(segments[0])#.GetOutput())#.GetOutputPort())
# cut_line_actor = vtk.vtkActor()
# cut_line_actor.SetMapper(cut_line_mapper)
# cut_line_actor.GetProperty().SetColor(colors.GetColor3d('Green'))

renderer = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)

axes = vtk.vtkAxesActor()
axes.SetShaftTypeToCylinder()
axes.SetCylinderRadius(0.05)
axes.SetTotalLength(2.5, 2.5, 2.5)


renderer.AddViewProp(mesh_actor)
renderer.AddViewProp(plane_actor)
renderer.AddViewProp(line_actor)
# renderer.AddViewProp(intersect_actor)
renderer.AddViewProp(axes)
# renderer.AddViewProp(clipper_actor)
# renderer.AddViewProp(cut_line_actor)
# renderer.AddViewProp(point_actor)
# renderer.AddViewProp(debug_line_actor)




renderer.SetBackground(colors.GetColor3d('White'))
renderer.GetActiveCamera().Pitch(90)
renderer.GetActiveCamera().SetViewUp(0, 0, 1)
renderer.ResetCamera()

renderWindow.SetSize(2000, 2000)
renderWindow.Render()
renderWindow.SetWindowName('ReadPolyData')
renderWindowInteractor.Start()