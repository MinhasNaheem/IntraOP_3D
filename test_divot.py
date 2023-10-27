import requests
import numpy as np
from MathFunctions import createTransformationMatrix,pointBasedRegistration
from CameraData import CameraData
import time
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

np.set_printoptions(suppress=True)
import os
vertex_divot = np.loadtxt('D:\IntraOP_3D\divot_vertex.txt')
center_divot= np.loadtxt('D:\IntraOP_3D\divot_centre.txt')
def label_points(points, ax):
    for i, (x, y, z) in enumerate(points):
        # (x,y,z) refers to the position where the text is to be placed
        ax.text(x, y, z,str(i + 1))



def updateData(toolMarker,refMarker,ax):
    cam = CameraData(toolMarker, refMarker)
    jsonData = cam.GetCameraData()
    markerData = cam.parseCameraData(jsonData)
    toolData = markerData.get(toolMarker, 0)
    refData = markerData.get(refMarker, 0)
    ref2cam_tf = []
    if refData != 0 and toolData != 0: 
        ref2cam_tf = createTransformationMatrix(refData[1], refData[0])
        tool2cam_tf = createTransformationMatrix(toolData[1], toolData[0])
        ref2tool_tf = np.linalg.inv(tool2cam_tf) @ ref2cam_tf
        tool2ref_tf = np.linalg.inv(ref2tool_tf)
        tooltip_position=tool2ref_tf[:3,3]
        reference_original=np.array([-83.9394,27.6742,56.7004],[-98.1793,67.7947,14.5623],[-18.6323,139.084,-1.74854],[23.069,81.072,71.306])
        reference_new=
        ax.scatter(tool2ref_tf[0,3],tool2ref_tf[1,3],tool2ref_tf[2,3],c='g', marker='o', label='tool tip position in the divot')
        tooltip2vertexdist = []
        for i in range(len(vertex_divot)):
            
            distance=np.linalg.norm(vertex_divot[i]-tooltip_position)
            tooltip2vertexdist.append(distance)
        vertex=vertex_divot[np.argmin(tooltip2vertexdist)]
        centre=center_divot[np.argmin(tooltip2vertexdist)]
        offset=np.linalg.norm(tool2ref_tf[:3,3]-vertex)
        dist_vector=centre-vertex
        print("offset between the vertex and the tool tip",offset)
        projection=((np.dot((tooltip_position-vertex),dist_vector)/np.square(np.linalg.norm(dist_vector))))
        # tipRadius=
        # pointOffset = (np.sqrt(2)-1)* tipRadius
        # offsetVector = np.array([pointOffset,0,0])
        print("projection",projection)
        ax.quiver(vertex[0],vertex[1],vertex[2],centre[0]-vertex[0],centre[1]-vertex[1],centre[2]-vertex[2])
        
        

if __name__ == "__main__":
    toolMarker = "11002"
    refMarker = "4321"
   
    while True:
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(vertex_divot[:,0],vertex_divot[:,1],vertex_divot[:,2],c='b', marker='o', label='Vertex Points')
        ax.scatter(center_divot[:,0],center_divot[:,1],center_divot[:,2], c='r', marker='o', label='Center Points')
        label_points(vertex_divot, ax)
        label_points(center_divot, ax)
        updateData(toolMarker,refMarker,ax)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')    
        ax.legend()
        ax.set_xlim([min(vertex_divot[:, 0]), max(vertex_divot[:, 0])])
        ax.set_ylim([min(vertex_divot[:, 1]), max(vertex_divot[:, 1])])
        ax.set_zlim([min(vertex_divot[:, 2]), max(vertex_divot[:, 2])])
        
        plt.show()
        # plt.pause(5)
        # plt.close()
        time.sleep(0.5)