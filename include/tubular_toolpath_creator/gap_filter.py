import vtk
from tubular_toolpath_creator.utils import renderVtkPolydata #debug


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
        self.center_line_size = center_line_size


    def addSegment(self, rotation_segement, index):

        number_of_cells = rotation_segement.GetLines().GetNumberOfConnectivityIds()
        p1_id = rotation_segement.GetLines().GetData().GetValue(number_of_cells)
        p2_id = rotation_segement.GetCell(0).GetPointId(0)
        p1 = rotation_segement.GetPoint(p1_id)
        p2 = rotation_segement.GetPoint(p2_id)

        # A  
        i = index
        if i == 0:
            #check which point is lower on z axis
            if p1[2] < p2[2]:
                self.old_end_pt = p2
                self.old_end_id = p2_id
            else:
                self.old_end_pt = p1
                self.old_end_id = p1_id
            self.old_rotation_segement = rotation_segement

        #B
        if i > 0 & i < self.center_line_size:
            p1_distance = vtk.vtkMath.Distance2BetweenPoints(self.old_end_pt, p1)
            p2_distance = vtk.vtkMath.Distance2BetweenPoints(self.old_end_pt, p2)

            if p1_distance < p2_distance:
                self.start_pt = p1
                self.end_pt = p2
                self.start_pt_id = p1_id
                self.end_pt_id = p2_id
            else:
                self.start_pt = p2
                self.end_pt = p1
                self.start_pt_id = p2_id
                self.end_pt_id = p1_id

            mx = ((self.old_end_pt[0] - self.start_pt[0]) / 2) + self.start_pt[0]
            my = ((self.old_end_pt[1] - self.start_pt[1]) / 2) + self.start_pt[1]
            mz = ((self.old_end_pt[2] - self.start_pt[2]) / 2) + self.start_pt[2]
            m = (mx, my, mz)

            rotation_segement.GetPoints().SetPoint(self.start_pt_id, m)
            self.old_rotation_segement.GetPoints().SetPoint(self.old_end_id, m)

            self.pre_combined_rotation_segment.AddInputData(self.old_rotation_segement)
            # renderVtkPolydata(self.old_rotation_segement)#debug #problem
            # renderVtkPolydata(self.pre_combined_rotation_segment.GetOutput())#debug #problem
            self.old_rotation_segement = rotation_segement
            self.old_end_id = self.end_pt_id
            self.old_end_pt = self.end_pt

    def getCombinedRotationSegement(self):
        strip = vtk.vtkStripper()
        self.pre_combined_rotation_segment.Update()
        strip.SetInputData(self.pre_combined_rotation_segment.GetOutput())
        strip.JoinContiguousSegmentsOn()
        # strip.SetMaximumLength(1)
        strip.Update()
        combined_rotation_segment = strip.GetOutput()
        return combined_rotation_segment