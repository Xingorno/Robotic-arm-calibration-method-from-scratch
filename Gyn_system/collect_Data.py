import os
f = open('c:/temp/20220426_data_collection_Kelowna.txt', 'w+')

def collectPoint(num):
  node = slicer.util.getNode('ProbeToReference')
  mat=vtk.vtkMatrix4x4()
  node.GetMatrixTransformToWorld(mat)
  f.write('DataSet_'+str(num)+ '\n' + \
  str(mat.GetElement(0,0)) + ' ' + str(mat.GetElement(0,1)) + ' ' +str(mat.GetElement(0,2)) + ' ' +str(mat.GetElement(0,3)) + ' \n' + \
  str(mat.GetElement(1,0)) + ' ' +str(mat.GetElement(1,1)) + ' ' +str(mat.GetElement(1,2)) + ' ' +str(mat.GetElement(1,3)) + ' \n' + \
  str(mat.GetElement(2,0)) + ' ' +str(mat.GetElement(2,1)) + ' ' +str(mat.GetElement(2,2)) + ' ' +str(mat.GetElement(2,3)) + ' \n' + \
  str(mat.GetElement(3,0)) + ' ' +str(mat.GetElement(3,1)) + ' ' +str(mat.GetElement(3,2)) + ' ' +str(mat.GetElement(3,3)) + '\n\n')
  
  f.flush()
  
os.fsync(f.fileno())

f.close()
