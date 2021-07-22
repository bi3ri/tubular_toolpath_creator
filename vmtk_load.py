import vtk
import pyvista as pv


def get_program_parameters():
    import argparse
    description = 'Read a polydata file.'
    epilogue = ''''''
    parser = argparse.ArgumentParser(description=description, epilog=epilogue,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    #parser.add_argument('filename', default='coil.vtp')
    args = parser.parse_args()
    args.filename = 'center_line.vtp'

    return args.filename


def main():
    colors = vtk.vtkNamedColors()

    filename = get_program_parameters()

    # Read all the data from the file
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(filename)
    reader.Update()
    output = reader.GetOutput()


    decimateFilter = vtk.vtkDecimatePolylineFilter()
    decimateFilter.SetTargetReduction(0.96)
    # decimateFilter.SetMaximumError(0.1)
    decimateFilter.SetInputData(output)
    decimateFilter.Update()
    # line = decimateFilter.GetOutput()

    # Visualize
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(decimateFilter.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(colors.GetColor3d('NavajoWhite'))

    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    renderer.AddActor(actor)
    renderer.SetBackground(colors.GetColor3d('DarkOliveGreen'))
    renderer.GetActiveCamera().Pitch(90)
    renderer.GetActiveCamera().SetViewUp(0, 0, 1)
    renderer.ResetCamera()

    renderWindow.SetSize(600, 600)
    renderWindow.Render()
    renderWindow.SetWindowName('ReadPolyData')
    renderWindowInteractor.Start()


if __name__ == '__main__':
    main()