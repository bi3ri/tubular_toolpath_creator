import vtk
import os

DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 


class DebugLines:

    def __init__(self, name):
        self.points = vtk.vtkPoints()
        self.cells = vtk.vtkCellArray()
        self.polydata = vtk.vtkPolyData()
        self.colors = vtk.vtkNamedColors()
        self.polydata_array = []
        self.actors = []
        self.name = name


    def addLine(self, p1, p2, color='Black'):
        self.points.InsertNextPoint(p1)
        self.points.InsertNextPoint(p2)

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

    def addPose(self, p, vx_norm, vy_norm, vz_norm):
        self.debug_x.addLine(p, (vx_norm * 0.01) + p)
        self.debug_y.addLine(p, (vy_norm * 0.01) + p)
        self.debug_z.addLine(p, (vz_norm * 0.01) + p)

    def saveVtp(self, path):
        self.debug_x.saveVtp(path)
        self.debug_y.saveVtp(path)
        self.debug_z.saveVtp(path)

# self.polydata_array.append(polydata)

# mapper = vtk.vtkPolyDataMapper()
# mapper.SetInputData(polydata)

# actor = vtk.vtkActor()
# actor.SetMapper(mapper)
# actor.GetProperty().SetColor(self.colors.GetColor3d(color))

# self.actors.append(actor)