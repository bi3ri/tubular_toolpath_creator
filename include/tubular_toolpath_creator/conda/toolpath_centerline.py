#!/usr/bin/python

import sys
import vtk
from vmtk import vtkvmtk
# from vmtk.vmtkcenterlines import vmtkCenterlines
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

class PointListSeedSelector():

    def __init__(self):
        self._Surface = None
        self._SeedIds = None
        self._SourceSeedIds = vtk.vtkIdList()
        self._TargetSeedIds = vtk.vtkIdList()
        self.PrintError = None
        self.PrintLog = None
        self.InputText = None
        self.OutputText = None
        self.InputInfo = None
        self.SourcePoints = None
        self.TargetPoints = None

    def SetSurface(self,surface):
        self._Surface = surface

    def GetSurface(self):
        return self._Surface

    def GetSourceSeedIds(self):
        return self._SourceSeedIds

    def GetTargetSeedIds(self):
        return self._TargetSeedIds

    def Execute(self):
    
        if not self._Surface:
            self.PrintError('vmtkPointListSeedSelector Error: Surface not set.')
            return

        if not self.SourcePoints:
            self.PrintError('vmtkPointListSeedSelector Error: SourcePoints not set.')
            return

        if not self.TargetPoints:
            self.PrintError('vmtkPointListSeedSelector Error: TargetPoints not set.')
            return

        self._SourceSeedIds.Initialize()
        self._TargetSeedIds.Initialize()

        if len(self.SourcePoints) % 3 != 0:
            self.PrintError('vmtkPointListSeedSelector Error: SourcePoints not made up of triplets.')
            return

        if len(self.TargetPoints) % 3 != 0:
            self.PrintError('vmtkPointListSeedSelector Error: TargetPoints not made up of triplets.')
            return

        pointLocator = vtk.vtkPointLocator()
        pointLocator.SetDataSet(self._Surface)
        pointLocator.BuildLocator()

        for i in range(len(self.SourcePoints)//3):
            point = [self.SourcePoints[3*i+0],self.SourcePoints[3*i+1],self.SourcePoints[3*i+2]]
            id = pointLocator.FindClosestPoint(point)
            self._SourceSeedIds.InsertNextId(id)

        for i in range(len(self.TargetPoints)//3):
            point = [self.TargetPoints[3*i+0],self.TargetPoints[3*i+1],self.TargetPoints[3*i+2]]
            id = pointLocator.FindClosestPoint(point)
            self._TargetSeedIds.InsertNextId(id)

class ToolpathCenterline:

    def __init__(self, input_mesh_path, output_mesh_path):
        print(input_mesh_path)
        print(output_mesh_path)
        self.capDisplacement = 0.0
        self.surface_mesh = loadVtp(input_mesh_path)
        self.output_mesh_path = output_mesh_path
        print(input_mesh_path)
        print(output_mesh_path)

    def capSurface(self):
        surface_capper = vtkvmtk.vtkvmtkCapPolyData()
        surface_capper.SetInputData(self.surface_mesh)
        surface_capper.SetDisplacement(self.capDisplacement)
        surface_capper.SetInPlaneDisplacement(self.capDisplacement)
        surface_capper.Update()
        return surface_capper.GetOutput(), surface_capper.GetCapCenterIds()

    def run(self):
        capped_surface, capCenterIds = self.capSurface()

        print(capCenterIds.GetNumberOfIds())
        assert((capCenterIds.GetNumberOfIds()) == 2)

        sourcePoint = capped_surface.GetPoint(capCenterIds.GetId(0))
        targetPoint = capped_surface.GetPoint(capCenterIds.GetId(1))


        pointLocator = vtk.vtkPointLocator()
        pointLocator.SetDataSet(capped_surface)
        pointLocator.BuildLocator()

        # sourceSeedIds = vtk.vtkIdList()
        # targetSeedIds = vtk.vtkIdList()

        # point = [self.SourcePoints[3*i+0],self.SourcePoints[3*i+1],self.SourcePoints[3*i+2]]
        id = pointLocator.FindClosestPoint(sourcePoint)
        print(id)
        sourceSeedId = [id]

        # point = [self.TargetPoints[3*i+0],self.TargetPoints[3*i+1],self.TargetPoints[3*i+2]]
        id = pointLocator.FindClosestPoint(targetPoint)
        print(id)
        targetSeedId = [id]

        vmtk_centerline_extractor = vmtkCenterlines() 
        vmtk_centerline_extractor.SeedSelectorName = 'pointlist'
        vmtk_centerline_extractor.Surface = capped_surface
        vmtk_centerline_extractor.SourcePoints = sourcePoint
        vmtk_centerline_extractor.TargetPoints = targetPoint
        # vmtk_centerline_extractor.SourceIds = sourceSeedId
        # vmtk_centerline_extractor.TargetIds = targetSeedId
        vmtk_centerline_extractor.Execute()
        centerline_output = vmtk_centerline_extractor.Centerlines
        saveVtp(self.output_mesh_path, centerline_output)
        print('done')


if __name__=='__main__':
    # toolpath_centerline = ToolpathCenterline('/home/bi3ri/tube_ws/src/tubular_toolpath_creator/data/clipped_coil.vtp', '/home/bi3ri/tube_ws/src/tubular_toolpath_creator/data/yey1.vtp')
    toolpath_centerline = ToolpathCenterline(sys.argv[1], sys.argv[2])
    toolpath_centerline.run()
