import numpy as np
from MathFunctions import createTransformationMatrix,pointBasedRegistration
from CameraData import CameraData
import time
import matplotlib.pyplot as plt
from functions import Registration
from mpl_toolkits import mplot3d

np.set_printoptions(suppress=True)
import os
vertex_divot = np.loadtxt('D:\IntraOP_3D\divot_vertex.txt')
center_divot= np.loadtxt('D:\IntraOP_3D\divot_centre.txt')
# vertex_divot = np.loadtxt('new_vertex_position.txt')
# center_divot= np.loadtxt('new_center_position.txt')
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
        return tooltip_position
        
        
toolMarker = "51001"
refMarker = "4321"
# divots in the original space
original_divots=np.array(([-14.3688,-0.104038,-1.81399],[0.182278,14.452,-1.83422],[-28.7293,28.8883,-1.5746]))
tool_positions = []
iteration_count=0
point_count_cmm = 4
while iteration_count<point_count_cmm:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(vertex_divot[:,0], vertex_divot[:,1], vertex_divot[:,2], c='b', marker='o', label='Vertex Points')
    ax.scatter(center_divot[:,0], center_divot[:,1], center_divot[:,2], c='r', marker='o', label='Center Points')
    label_points(vertex_divot, ax)
    label_points(center_divot, ax)
    tooltip_position = updateData(toolMarker, refMarker, ax)
    
    # Store the first three tool positions
    if len(tool_positions) < point_count_cmm:
        tool_positions.append(tooltip_position)
        
    # np.savetxt("tool_tip_position.csv",tool_positions)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')    
    ax.legend()
    ax.set_xlim([min(vertex_divot[:, 0]), max(vertex_divot[:, 0])])
    ax.set_ylim([min(vertex_divot[:, 1]), max(vertex_divot[:, 1])])
    ax.set_zlim([min(vertex_divot[:, 2]), max(vertex_divot[:, 2])])
    
    plt.show()
    time.sleep(0.5)
    iteration_count +=1 
    
# data=np.loadtxt(r"D:\IntraOP_3D\tool_tip_position.csv")
tf,err=Registration(vertex_divot[:point_count_cmm,:],tool_positions)
print("resgitration error=",err)
# new_vertex_position=(tf@(np.concatenate((vertex_divot, np.ones((18,1))), axis=1)).T).T
# new_center_position=(tf@(np.concatenate((center_divot, np.ones((18,1))), axis=1)).T).T
# np.savetxt("new_vertex_position.txt",new_vertex_position[:,0:3])
# np.savetxt("new_center_position.txt",new_center_position[:,0:3])

