import numpy as np
from MathFunctions import *
import pandas as pd

CTPoints = np.loadtxt('RegistrationPoints\\ct_cmm.txt')
CMMPoints = np.loadtxt('RegistrationPoints\\phantom_cmm.txt')


df = pd.read_csv('4330_collect_batch2_autoreg_hel2Referecnce.csv')

ref_pos = df[['refx','refy','refz']].to_numpy()
ref_quat = df [['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
phantomPos = df[['toolx','tooly','toolz']].to_numpy()
phantomQuat = df [['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()

ref_pos, ref_quat = filterData(ref_pos, ref_quat) 
phantomPos, phantomQuat = filterData(phantomPos, phantomQuat)

def GenerateRef2CT(refPos:np.array, refQuat:np.array, 
                   phantomPos:np.array, phantomQuat:np.array, 
                   cmmPoints:np.array, ctPoints:np.array):
    ref2cam_tf = createTransformationMatrix(refPos, refQuat)
    phantom2Cam_tf = createTransformationMatrix(phantomPos, phantomQuat)
    phantom2Ct, err = pointBasedRegistration(cmmPoints, ctPoints)
    ref2phantom_tf = np.linalg.inv(phantom2Cam_tf) @ ref2cam_tf
    ref2Ct_tf = ref2phantom_tf @ phantom2Ct
    
    return ref2Ct_tf

ref2ct_tf = GenerateRef2CT(ref_pos, ref_quat, phantomPos, 
                           phantomQuat, CMMPoints, CTPoints)

np.save('ref2ct_autoreg', ref2ct_tf)
   