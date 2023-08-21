



""" The aim is to perform registration between the imaging coordinate system 
of the Carm and the reference marker attached to Carm 
PreReq:
The precalibration CT loaction has to be recorded and updated

"""
###################
import numpy as np
from MathFunctions import *
from functions import plot_fids
import matplotlib.pyplot as plt
import pandas as pd
import SimpleITK as sitk
from functions import sphere_detect
# Load the image.
reader = sitk.ImageSeriesReader()
dicom_dir = r"D:\Navigation\Carm_registration\Dataset-3-20230610T054412Z-001\Dataset-3\DICOM\PA0\ST0\SE1"
dicom_names = reader.GetGDCMSeriesFileNames(dicom_dir)
reader.SetFileNames(dicom_names)
image = reader.Execute()
CT_detected_points = sphere_detect(image)


CMMPoints = np.loadtxt('RegistrationPoints\\phantom_cmm.txt')


# The tool marker is the helical(phantom) and the reference marker is the C-Arm marker.

df = pd.read_csv('4330_collect_batch3_metal_Oarm.csv')
Carm_pos = df[['refx','refy','refz']].to_numpy()
Carm_quat = df [['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
phantomPos = df[['toolx','tooly','toolz']].to_numpy()
phantomQuat = df [['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()

CArmPos, CArmQuat = filterData(Carm_pos, Carm_quat) 
phantomPos, phantomQuat = filterData(phantomPos, phantomQuat)


def ComputeCT2CArm (CTpoints: np.array , CMMPoints: np.array, 
                        phantomPos:np.array, phantomQuat:np.array, 
                        CArmPos:np.array, CArmQuat:np.array):
    CT2Phantom_tf , error = registrationWithoutCorrespondence(
                                                CTpoints, CMMPoints)
   
    Phantom2Cam_tf = createTransformationMatrix(phantomPos, phantomQuat)
    CArm2Cam_tf = createTransformationMatrix(CArmPos, CArmQuat)
    CArm2Phantom_tf = np.linalg.inv(Phantom2Cam_tf) @ CArm2Cam_tf 
    # CT2phantom is the ICP registration output matrix
    CArm2CT_tf = np.linalg.inv(CT2Phantom_tf) @ CArm2Phantom_tf 
    return np.linalg.inv(CArm2CT_tf)

CT2CArm_tf = ComputeCT2CArm (CT_detected_points, CMMPoints, phantomPos, 
                             phantomQuat, CArmPos, CArmQuat)

np.save('CT2CArm',CT2CArm_tf)