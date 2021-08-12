import vtk
import numpy as np



colors = vtk.vtkNamedColors()




filename = 'coil_full.ply'
coil_ply = vtk.vtkPLYReader()
coil_ply.SetFileName(filename)
coil_ply.Update()

cleanFilter = vtk.vtkCleanPolyData()
cleanFilter.SetInputConnection(coil_ply.GetOutputPort())
cleanFilter.SetAbsoluteTolerance(0.1)
cleanFilter.Update()

delaunay3D = vtk.vtkDelaunay3D()
delaunay3D.SetTolerance(0.1)
delaunay3D.SetAlpha(0.01)
delaunay3D.SetInputConnection(cleanFilter.GetOutputPort())

surface = vtk.vtkDataSetSurfaceFilter()
surface.SetInputData(delaunay3D.GetOutput())
surface.Update()






mesh_mapper = vtk.vtkPolyDataMapper()
mesh_mapper.SetInputConnection(delaunay3D.GetOutputPort())
mesh_actor = vtk.vtkActor()
mesh_actor.SetMapper(mesh_mapper)
mesh_actor.GetProperty().SetColor(colors.GetColor3d('Red'))



renderer = vtk.vtkRenderer()
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)


renderer.AddViewProp(mesh_actor)

renderer.SetBackground(colors.GetColor3d('White'))
renderer.GetActiveCamera().Pitch(90)
renderer.GetActiveCamera().SetViewUp(0, 0, 1)
renderer.ResetCamera()

renderWindow.SetSize(600, 600)
renderWindow.Render()
renderWindow.SetWindowName('ReadPolyData')
renderWindowInteractor.Start()