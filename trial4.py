import numpy as np
from scipy.spatial.transform import Rotation as R
from functions import *
np.set_printoptions(suppress=True)

# define rotation parameters
theta = np.pi/3
r = R.from_quat([0, 0, np.sin(theta), np.cos(theta)])

A = np.array([[49.8782,-60,3.487824],[-49.72609,60,-5.226423],[-23.47357814,60,-44.14737964],[-12.09609478,-10,-48.51478631],[-40.45084972,-10,29.38926261],[-48.51478631,-60,12.09609478],[10.39558454,50,-48.90738004]])

B = rotate_vec(r,A)+np.random.rand(len(A),3)
# random_pts = np.random.rand(2,3)*10
# B = np.vstack((random_pts,B))
# B = B[[3,0,8,1,2,5,7],:]
B = B[[5,6,1,4,0,3,2],:]

intersec_ip, tf, reg_err= correspondence(A,B)

plotpts_A = plot_fids(A).show()
plotpts_B = plot_fids(B).show()

print("Wait")