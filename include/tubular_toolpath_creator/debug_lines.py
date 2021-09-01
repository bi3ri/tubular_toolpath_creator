import vtk


class DebugLines:

    def __init__(self, name):
        self.idx = 0
        self.points = vtk.vtkPoints()
        self.cells = vtk.vtkCellArray()
        self.polydata = vtk.vtkPolyData()
        # self.actor = vtk.vtkActor()
        self.colors = vtk.vtkNamedColors()

        self.polydata_array = []
        self.actors = []
        self.name = name

    
    def addLine(self, p1, p2, color='Black'):
        points = vtk.vtkPoints()
        points.InsertNextPoint(p1)
        points.InsertNextPoint(p2)

        polyline = vtk.vtkPolyLine()
        polyline.GetPointIds().SetNumberOfIds(2)
        polyline.GetPointIds().SetId(0, 0)
        polyline.GetPointIds().SetId(1, 1)

        cell = vtk.vtkCellArray()
        cell.InsertNextCell(polyline)
        self.cells.InsertNextCell(polyline)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(cell)

        self.polydata_array.append(polydata)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d(color))

        self.actors.append(actor)

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
        writer.SetFilePrefix(os.path.join(data_path, name))
        writer.Write ()

    def saveVtp(self, path):
        idx = 0
        writer = vtk.vtkXMLPolyDataWriter()
        for polydat in self.polydata_array:
            writer.SetFileName(path + '/debug/' + str(idx) + 'debug_'+ self.name +  '.vtp')
            idx+=1
            # writer.vtkSetFilePathMacro(path)
            writer.SetInputData(polydat)
            # writer.Update()
            writer.Write()