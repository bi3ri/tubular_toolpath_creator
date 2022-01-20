#!/usr/bin/python

import sys
import vtk
from vmtk import vtkvmtk
from vmtkcenterlines import vmtkCenterlines

def saveVtp(path, mesh):
    writer = vtk.vtkXMLPolyDataWriter()
    writer.SetFileName(path)
    writer.SetInputData(mesh)
    writer.Write()

def loadVtp(path):
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(path)
    reader.Update()
    return reader.GetOutput()

class ToolpathCenterline:

    def __init__(self, input_mesh_path, output_mesh_path):
        self.capDisplacement = 0.0
        self.surface_mesh = loadVtp(input_mesh_path)
        self.output_mesh_path = output_mesh_path

    def capSurface(self):
        surface_capper = vtkvmtk.vtkvmtkCapPolyData()
        surface_capper.SetInputData(self.surface_mesh)
        surface_capper.SetDisplacement(self.capDisplacement)
        surface_capper.SetInPlaneDisplacement(self.capDisplacement)
        surface_capper.Update()
        return surface_capper.GetOutput(), surface_capper.GetCapCenterIds()

    def run(self):
        capped_surface, capCenterIds = self.capSurface()
        assert((capCenterIds.GetNumberOfIds()) == 2)

        sourcePoint = capped_surface.GetPoint(capCenterIds.GetId(0))
        targetPoint = capped_surface.GetPoint(capCenterIds.GetId(1))

        pointLocator = vtk.vtkPointLocator()
        pointLocator.SetDataSet(capped_surface)
        pointLocator.BuildLocator()

        vmtk_centerline_extractor = vmtkCenterlines() 
        vmtk_centerline_extractor.SeedSelectorName = 'pointlist'
        vmtk_centerline_extractor.Surface = capped_surface
        vmtk_centerline_extractor.SourcePoints = sourcePoint
        vmtk_centerline_extractor.TargetPoints = targetPoint
        vmtk_centerline_extractor.Execute()
        centerline_output = vmtk_centerline_extractor.Centerlines
        saveVtp(self.output_mesh_path, centerline_output)

if __name__=='__main__':
    toolpath_centerline = ToolpathCenterline(sys.argv[1], sys.argv[2])
    toolpath_centerline.run()
