from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader
from vtkmodules.vtkCommonDataModel import vtkUnstructuredGrid
import numpy as np
import transformations as t1

Twc=np.array([[ 8.66025346e-01, -2.50000094e-01 , 4.33012828e-01, 6.43301262e-01],[5.00000112e-01,4.33012724e-01,-7.49999974e-01,2.49999916e-02],
              [8.72695997e-09,8.66025442e-01,5.00000064e-01,6.50000068e-01],[0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])

Tcw=np.linalg.inv(Twc)
Q=t1.quaternion_from_matrix(Twc)
print(Q)
Two=np.array([[1,0,0,0.3],[0,1,0,0.25],[0,0,1,0.5],[0,0,0,1]])

Tco=np.dot(Tcw,Two)

def main():

    # read vtk unstructured grid
    reader = vtkUnstructuredGridReader()
    reader.SetFileName("sola_bene_m_tracker.vtk")
    reader.Update()
    grid: vtkUnstructuredGrid = reader.GetOutput()

    # store all points in the vtk grid output in a list
    points = []
    
    for i in range(grid.GetNumberOfPoints()):
        point = grid.GetPoint(i)
        for j in range(3):
            points.append(float(point[j]))
            if j==2:
                
                points.append(1)
    
    # divide each point by 1000

    points_f=[]

    for i in range(0,len(points),4):
          pt=np.array([points[i],points[i+1],points[i+2],points[i+3]])
          tmp=np.array(np.dot(Two,pt))
          points_f=points_f+list(np.delete(tmp,3))



    # save list to file
    with open("vertex.txt", "w") as f:
        for item in points_f:
            f.write("%s\n" % item)

    # for each cell print in the vtk grid print id of vertieces
    with open("triangle.txt", "w") as f:
        for i in range(grid.GetNumberOfCells()):
            cell = grid.GetCell(i)
            for j in range(cell.GetNumberOfPoints()):
                f.write("%s\n" % cell.GetPointId(j))


if __name__ == '__main__':
    main()
