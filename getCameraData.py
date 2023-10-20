import requests
import numpy as np
from MathFunctions import createTransformationMatrix,pointBasedRegistration
from CameraData import CameraData
import time
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
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
        ax.scatter(tool2ref_tf[0,3],tool2ref_tf[1,3],tool2ref_tf[2,3],c='g', marker='o', label='tool tip position in the divot')
        tooltip2vertexdist = []
        for i in range(len(vertex_divot)):
            
            distance=np.linalg.norm(vertex_divot[i]-tooltip_position)
            tooltip2vertexdist.append(distance)
        min_dist_vertex=vertex_divot[np.argmin(tooltip2vertexdist)]
        min_dist_centre=center_divot[np.argmin(tooltip2vertexdist)]
        error=np.linalg.norm(tool2ref_tf[:3,3]-min_dist_vertex)
        print("offset between the vertex and the tool tip",error)
        ax.quiver(min_dist_vertex[0],min_dist_vertex[1],min_dist_vertex[2],min_dist_centre[0]-min_dist_vertex[0],min_dist_centre[1]-min_dist_vertex[1],min_dist_centre[2]-min_dist_vertex[2])
        
        # print(tool2ref_tf)

if __name__ == "__main__":
    toolMarker = "11002"
    refMarker = "4321"
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim3d(0, 15)
    ax.set_ylim3d(0, 15)
    ax.set_zlim3d(0, 15)
    
    
    def update(frame):
        ax.clear()
        ax.scatter(vertex_divot[:,0], vertex_divot[:,1], vertex_divot[:,2], c='b', marker='o', label='Vertex Points')
        ax.scatter(center_divot[:,0], center_divot[:,1], center_divot[:,2], c='r', marker='o', label='Center Points')
        label_points(vertex_divot, ax)
        label_points(center_divot, ax)
        updateData(toolMarker, refMarker, ax)
        
    
        ax.legend()
    
    ani = FuncAnimation(fig, update, frames=100, interval=500)  # Adjust frames and interval as needed
    plt.show()
   
        
        

