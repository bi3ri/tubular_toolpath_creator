import vtk
import os
import pyvista as pv
from tubular_toolpath_creator.utils import *

DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 

class DebugLines:

    def __init__(self, name, scaling_factor=1.0):
        self.points = vtk.vtkPoints()
        self.cells = vtk.vtkCellArray()
        self.polydata = vtk.vtkPolyData()
        self.colors = vtk.vtkNamedColors()
        self.polydata_array = []
        self.actors = []
        self.name = name
        self.scaling_factor = scaling_factor

    def addLine(self, start_point, end_point):
        if self.scaling_factor != 1.0:
            v = end_point - start_point
            v = normalize(v) * self.scaling_factor
            
            end_point = start_point + v

        self.points.InsertNextPoint(start_point)
        self.points.InsertNextPoint(end_point)


    def setLines(self):
        number_of_points = self.points.GetNumberOfPoints()
        idx = 0
        for i in range(number_of_points//2):
            line = vtk.vtkLine()
            line.GetPointIds().SetNumberOfIds(2)
            line.GetPointIds().SetId(0, idx) 
            idx += 1
            line.GetPointIds().SetId(1, idx) 
            idx += 1
            self.cells.InsertNextCell(line)
        self.polydata.SetPoints(self.points)
        self.polydata.SetLines(self.cells)

        # self.polydata_array.append(polydata)

    def render(self):
        renderer = vtk.vtkRenderer()
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.SetWindowName('DebugLines')
        renderWindow.SetSize(2000, 2000)
        renderWindow.AddRenderer(renderer)
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        for actor in self.actors:
            renderer.AddActor(actor)
        renderer.SetBackground(self.colors.GetColor3d('White'))

        renderWindow.Render()
        renderWindowInteractor.Start()    

    def saveOBJ(self, name):
        renderer = vtk.vtkRenderer()
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.SetWindowName('DebugLines')
        renderWindow.SetSize(2000, 2000)
        renderWindow.AddRenderer(renderer)

        for actor in self.actors:
            renderer.AddActor(actor)
        renderer.SetBackground(self.colors.GetColor3d('White'))

        # renderWindow.Render()

        writer = vtk.vtkOBJExporter()
        writer.SetInput(renderWindow)
        writer.SetFilePrefix(os.path.join(DATA_PATH, name))
        writer.Write ()

    def saveVtp(self, path):
        self.setLines()
        writer = vtk.vtkXMLPolyDataWriter()
        writer.SetFileName(os.path.join(path, ('debug/' + self.name + '.vtp')))
        writer.SetInputData(self.polydata)
        writer.Write()

class DebugPoses:

    def __init__(self, name):
        self.debug_x = DebugLines('debug_poses/' + name + '_x')
        self.debug_y = DebugLines('debug_poses/' + name + '_y')
        self.debug_z = DebugLines('debug_poses/' + name + '_z')
        self.name = name

    def addPose(self, p, rotation_matrix):
        self.debug_x.addLine(p, (rotation_matrix[:3,:3] @ [1, 0, 0] * 0.01) + p)
        self.debug_y.addLine(p, (rotation_matrix[:3,:3] @ [0, 1, 0] * 0.01) + p)
        self.debug_z.addLine(p, (rotation_matrix[:3,:3] @ [0, 0, 1] * 0.01) + p)

    def saveVtp(self, path):
        self.debug_x.saveVtp(path)
        self.debug_y.saveVtp(path)
        self.debug_z.saveVtp(path)

class DebugPoints:
    def __init__(self):
        self.points = vtk.vtkPoints()

    def addPoint(self, p):
        self.points.InsertNextPoint(p)

    def save(self, path):
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(self.points)

        writer = vtk.vtkXMLPolyDataWriter()
        writer.SetFileName(path)
        writer.SetInputData(polydata)
        writer.Write()
    
def saveDebugLine(p1, p2, path):
    points = vtk.vtkPoints()
    points.InsertNextPoint(p1)
    points.InsertNextPoint(p2)

    line = vtk.vtkLine()
    line.GetPointIds().SetNumberOfIds(2)
    line.GetPointIds().SetId(0, 0) 
    line.GetPointIds().SetId(1, 1) 

    cell = vtk.vtkCellArray()
    cell.InsertNextCell(line)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetLines(cell)

    writer = vtk.vtkXMLPolyDataWriter()
    writer.SetFileName(path)
    writer.SetInputData(polydata)
    writer.Write()


def saveDebugPlane(origin, normal, path):
    pv.Plane(origin, normal).save(path)
