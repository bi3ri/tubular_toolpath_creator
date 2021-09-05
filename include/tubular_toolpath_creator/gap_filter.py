import vtk
# from tubular_toolpath_creator.utils import renderVtkPolydata #debug
import tubular_toolpath_creator.utils
import os
import numpy as np
DATA_PATH = os.path.normpath(os.path.join(os.path.dirname(__file__), '../../data')) 

class GapFilter:

    start_pt = None
    end_pt = None
    start_pt_id = 0
    end_pt_id = 0
    old_end_pt = None
    old_end_id = 0
    output = None
    old_rotation_segement = None
    center_line_size = 0
    pre_combined_rotation_segment = vtk.vtkAppendPolyData()

    def __init__(self, center_line_size):
        self.start_pt = None
        self.end_pt = None
        self.start_pt_id = 0
        self.end_pt_id = 0
        self.old_end_pt = None
        self.old_end_id = 0
        self.output = None
        self.old_rotation_segement = None
        self.center_line_size = 0
        self.pre_combined_rotation_segment = vtk.vtkAppendPolyData()
        self.center_line_size = center_line_size
        self.list = []
        self.is_initial_element = True

    def reorderPolyline(self, start_pt_id, rotation_segement):

        number_of_points = rotation_segement.GetPoints().GetNumberOfPoints()
        orderd_segment = vtk.vtkPolyLine()
        orderd_segment.GetPointIds().SetNumberOfIds(number_of_points)
        points = vtk.vtkPoints()

        idx = 0
        for i in range(start_pt_id, -1, -1):
            points.InsertNextPoint(rotation_segement.GetPoint(i))
            orderd_segment.GetPointIds().SetId(idx, idx)
            idx += 1
        
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(orderd_segment)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(cells)

        return polydata

    def addSegment(self, rotation_segement):
        number_of_ids = rotation_segement.GetLines().GetNumberOfConnectivityIds()

        #if rotation segement is empty return
        if number_of_ids == 0:
            return

        p1_id = rotation_segement.GetCell(0).GetPointId(0)
        p2_id = rotation_segement.GetLines().GetData().GetValue(number_of_ids)
        p1 = rotation_segement.GetPoint(p1_id)
        p2 = rotation_segement.GetPoint(p2_id)

        #initial segment
        if self.is_initial_element:
            #check which point is lower on z axis
            if p1[2] < p2[2]:
                self.old_end_pt = p2
                self.old_end_id = p2_id
            else:
                self.old_end_pt = p1
                self.old_end_id = p1_id
            self.old_rotation_segement = rotation_segement
            self.is_initial_element = False
            return

        #noninitial element
        p1_distance = vtk.vtkMath.Distance2BetweenPoints(self.old_end_pt, p1)
        p2_distance = vtk.vtkMath.Distance2BetweenPoints(self.old_end_pt, p2)

        if p1_distance < p2_distance:
            self.start_pt = p1
            self.end_pt = p2
            self.start_pt_id = p1_id
            self.end_pt_id = p2_id

        else:
            rotation_segement = self.reorderPolyline(p2_id, rotation_segement)

            self.start_pt_id = 0
            self.end_pt_id = rotation_segement.GetPoints().GetNumberOfPoints() - 1
            self.start_pt = rotation_segement.GetPoint(self.start_pt_id)
            self.end_pt = rotation_segement.GetPoint(self.end_pt_id)

        # assert(self.end_pt_id == 0 or self.start_pt_id != 0)
        
        mx = ((self.old_end_pt[0] - self.start_pt[0]) / 2) + self.start_pt[0]
        my = ((self.old_end_pt[1] - self.start_pt[1]) / 2) + self.start_pt[1]
        mz = ((self.old_end_pt[2] - self.start_pt[2]) / 2) + self.start_pt[2]
        m = np.asarray([mx, my, mz], dtype=np.float64)

        rotation_segement.GetPoints().SetPoint(self.start_pt_id, m)
        self.old_rotation_segement.GetPoints().SetPoint(self.old_end_id, self.old_end_pt)
        self.pre_combined_rotation_segment.AddInputData(self.old_rotation_segement)
        self.list.append(self.old_rotation_segement)
        self.old_rotation_segement = rotation_segement
        self.old_end_id = self.end_pt_id
        self.old_end_pt = self.end_pt
    
    def getSegments(self):
        self.list.append(self.old_rotation_segement)
        return self.list

    def getCombinedRotationSegement(self):
        self.pre_combined_rotation_segment.Update()
        decimateFilter2 = vtk.vtkDecimatePolylineFilter()
        decimateFilter2.SetTargetReduction(0.1)
        decimateFilter2.SetInputData(self.pre_combined_rotation_segment.GetOutput())
        decimateFilter2.Update()

        clean = vtk.vtkCleanPolyData()
        clean.SetTolerance(0.01)
        clean.PointMergingOn()
        clean.SetInputData(decimateFilter2.GetOutput())
        clean.Update()

        strip = vtk.vtkStripper()
        strip.SetInputData(clean.GetOutput())
        strip.JoinContiguousSegmentsOn()
        strip.Update()
        return strip.GetOutput()