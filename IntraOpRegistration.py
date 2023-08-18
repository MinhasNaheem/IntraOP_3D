import numpy as np
from MathFunctions import *
import pandas as pd

df = pd.read_csv('54320_collect_batch4_ref_Oarm.csv')

CArm_pos = df[['refx','refy','refz']].to_numpy()
CArm_quat = df [['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
ref_pos = df[['toolx','tooly','toolz']].to_numpy()
ref_quat = df [['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()

CT2CArm_tf = np.load('CT2CArm.npy')
CArmPos, CArmQuat = filterData(CArm_pos, CArm_quat) 
refPos, refQuat = filterData(ref_pos, ref_quat)

def computeRef2CT(refPos:np.array, refQuat:np.array, 
                  CArmPos:np.array, CArmQuat:np.array, 
                  CT2Carm_tf:np.array):

    ref2cam_tf = createTransformationMatrix(refPos, refQuat)
    Carm2Cam_tf = createTransformationMatrix(CArmPos, CArmQuat)
    ref2Carm_tf = np.linalg.inv(Carm2Cam_tf) @ ref2cam_tf
    ref2ct_tf = np.linalg.inv(CT2Carm_tf) @ ref2Carm_tf 
    
    return ref2ct_tf 

ref2CT_tf = computeRef2CT(refPos, refQuat, CArmPos, 
                          CArmQuat, CT2CArm_tf)

print(ref2CT_tf)

np.save('ref2CT',ref2CT_tf)
  