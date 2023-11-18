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
import keyboard
offset_errors = [] 
# load the cmm points of the vertex and centre
# vertex_divot = np.loadtxt('new_vertex_position.txt')
# center_divot= np.loadtxt('new_center_position.txt')

vertex_divot=np.loadtxt("D:\\IntraOP_3D\\ASTM_data\\vertex.txt")
center_divot=np.loadtxt("D:\\IntraOP_3D\\ASTM_data\\center.txt")
# function to label the points in the plot
def label_points(points, ax):
    for i, (x, y, z) in enumerate(points):
        ax.text(x, y, z,str(i + 1))


# function for finding tooltip position
def updateData(toolMarker,refMarker):
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
        tooltip2vertexdist = []
        for i in range(len(vertex_divot)):
            
            distance=np.linalg.norm(vertex_divot[i]-tooltip_position)
            tooltip2vertexdist.append(distance)
            
        vertex=vertex_divot[np.argmin(tooltip2vertexdist)]
        centre=center_divot[np.argmin(tooltip2vertexdist)]
        offset=np.linalg.norm(tool2ref_tf[:3,3]-vertex)
        dist_vector=centre-vertex
        projection=((np.dot((tooltip_position-vertex),dist_vector)/np.square(np.linalg.norm(dist_vector))))
        # pointOffset = (np.sqrt(2)-1)* tipRadius
        # offsetVector = np.array([pointOffset,0,0])
        # print("projection/displacement of the tip from the axis of the divot",projection)
        return tooltip_position
        
# keyboard event 
def on_key_event(e):
    global tooltip_position_vertex,vertex,center,offset_correction
    divot_index = e - 1
    if 0 <= divot_index < len(vertex_divot):
        vertex = vertex_divot[divot_index]
        center = center_divot[divot_index]
        tooltip_position_vertex = updateData(toolMarker, refMarker)
        offset_errors.append(np.linalg.norm(tooltip_position_vertex - vertex))
        a=center-tooltip_position_vertex
        print(f"Divot {divot_index + 1}:")
        print(f"tooltip placed in vertex:{tooltip_position_vertex}")
        print(f"Offset between the tooltip and the vertex: {np.linalg.norm(tooltip_position_vertex - vertex)}")
        # plot()
        # time.sleep(5)
        
        # tooltip_position_in_center=updateData(toolMarker,refMarker)
        # r=tooltip_position_in_center-center
        # offset_correction=a+r
        # print(f"tooltip placed in center:{tooltip_position_in_center}")
        # print(f"offset correction for Divot {divot_index+1}",offset_correction)
        # return offset_correction
        
    
def plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=30, azim=45, roll=0)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim3d(vertex[0]+5, vertex[0]-5)
    ax.set_ylim3d(vertex[1]+5, vertex[1]-5)
    ax.set_zlim3d(vertex[2]+5, vertex[2]-5)
    
    ax.scatter(vertex[0], vertex[1], vertex[2], c='b', marker='o', label='Vertex Points')
    ax.scatter(center[0], center[1], center[2], c='r', marker='o', label='Center Points')
    ax.quiver(vertex[0],vertex[1],vertex[2],center[0]-vertex[0],center[1]-vertex[1],center[2]-vertex[2])
    ax.scatter(tooltip_position_vertex[0],tooltip_position_vertex[1],tooltip_position_vertex[2],c='g', marker='o', label='tool tip position in the divot')
    plt.show()
    return fig

if __name__ == "__main__":
    # toolMarker = "11002"
    toolMarker = "51001"
    refMarker = "4321"
    iteration_count=0
    number_of_divots = 18
    while iteration_count<number_of_divots:
        x =int(input("press a key between 1 to 18: "))
        # tooltip_position_in_vertex=updateData(toolMarker,refMarker)
        # print("tool tip position in divot vertex=",tooltip_position_in_vertex)
        # a=center_divot-tooltip_position_in_vertex
        # print(a)
        # time.sleep(3)
        # tooltip_position_in_center=updateData(toolMarker,refMarker)
        # print("tool tip position in divot center=",tooltip_position_in_center)
        # r=tooltip_position_in_center-center_divot
        # offset_correction=a+r
        # vertex,center,offset_from_vertex=on_key_event(x)
        
        offset_correction=on_key_event(x)
        plot()
        iteration_count +=1 
mean_offset=np.mean(offset_errors)
# print("mean offset from the vertex :",mean_offset)
# print("offset correction : ",offset_correction)


