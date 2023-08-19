from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist, euclidean
from scipy.spatial import distance_matrix
import numpy as np
import SimpleITK as sitk
import open3d as o3d
    
def createPoseMatrix(rotationMatrix:np.array, 
                        positionVector:np.array):
    
    temp = np.column_stack((rotationMatrix,positionVector))
    transformationMatrix = np.vstack((temp,[0,0,0,1]))
    
    return transformationMatrix

def createTransformationMatrix(pos:np.array, quat:np.array):
    
    r_marker2cam = R.from_quat(quat).as_matrix().transpose()
    tf_marker2cam = createPoseMatrix(r_marker2cam,pos)
    
    return tf_marker2cam

def transformPoints(points:np.array, transformationMatrix:np.array):
    
    pointsHomogenous = np.hstack((points,np.ones((len(points),1))))
    transformedPointsHm = (transformationMatrix @ 
                            pointsHomogenous.transpose())
    transformedPoints = transformedPointsHm.transpose()[:,:3]
    
    return transformedPoints

def CTtoVTK(dicomFilesPath:str):
    
    dicomReader = sitk.ImageSeriesReader()
    dicomNames = dicomReader.GetGDCMSeriesFileNames(dicomFilesPath)
    dicomReader.SetFileNames(dicomNames)
    dicomImage = dicomReader.Execute()
    
    ImageDim = np.array(list(dicomImage.GetSize()))
    ImageDir = np.array(list(dicomImage.GetDirection()))
    ImageOrient = np.array(ImageDir).reshape(3,3)
    ImageSpacing = np.array(list(dicomImage.GetSpacing()))
    volumeCT = (ImageDim-1)*ImageSpacing
    CTOrigin = np.array(dicomImage.GetOrigin())
    
    positionVector = volumeCT + ImageOrient @ CTOrigin
    alteredImageOrient = ImageOrient @ np.array([1,0,0,
                                                    0,-1,0,
                                                    0,0,-1]).reshape(
                                                            3,3)    
    transformationCTtoVTK = createPoseMatrix(alteredImageOrient,
                                             positionVector)
    
    
    return transformationCTtoVTK

def geometricMedianFilter(X, eps=0.05):
    y = np.mean(X, 0)
    while True:
        D = cdist(X, [y])
        nonzeros = (D != 0)[:, 0]
        Dinv = 1 / D[nonzeros]
        Dinvs = np.sum(Dinv)
        W = Dinv / Dinvs
        T = np.sum(W * X[nonzeros], 0)
        num_zeros = len(X) - np.sum(nonzeros)
        if num_zeros == 0:
            y1 = T
        elif num_zeros == len(X):
            return y
        else:
            R = (T - y) * Dinvs
            r = np.linalg.norm(R)
            rinv = 0 if r == 0 else num_zeros/r
            y1 = max(0, 1-rinv)*T + min(1, rinv)*y

        if euclidean(y, y1) < eps:
            return y1

        y = y1

def filterData(position:np.array, quaternion:np.array):
    
    quaternionFiltered = R.from_quat(quaternion).mean().as_quat()
    positionMedian = geometricMedianFilter(position)
    
    return positionMedian, quaternionFiltered

def estimatePointToPointError(sourcePoints:np.array, 
                              targetPoints:np.array):
    error = []
    for i in range(len(sourcePoints)):
        min_error = float("inf")
        relative_difference = np.abs(targetPoints - sourcePoints[i])  
        for k in range(len(relative_difference)):
            if relative_difference[k] < min_error:
                min_error = relative_difference[k]
        error.append(min_error)
    return np.array(error)

def pointBasedRegistration(sourcePoints:np.array, 
                           targetPoints:np.array):
    src_pointCloud = o3d.geometry.PointCloud()
    src_pointCloud.points = o3d.utility.Vector3dVector(sourcePoints)
    tgt_pointCloud = o3d.geometry.PointCloud()
    tgt_pointCloud.points = o3d.utility.Vector3dVector(targetPoints)
    ind = np.arange(len(sourcePoints))
    cor = np.hstack((ind,ind))
    
    corres_mat = np.vstack((ind,ind)).transpose()
    p2p = (o3d.pipelines.registration
           .TransformationEstimationPointToPoint())
    transformation_mat = p2p.compute_transformation(
                                src_pointCloud, 
                                tgt_pointCloud,
                                o3d.utility.Vector2iVector(corres_mat))
    ErrorCalc_vec = []
    Src_CtPts = np.asarray(src_pointCloud.points)
    for i in range(len(Src_CtPts)):
        errCal_vec = list(Src_CtPts[i])
        errCal_vec.append(1)
        errCal_vec2 = np.dot(transformation_mat,errCal_vec)
        errCal_vec3 = np.asarray(errCal_vec2[0:3])
        ErrorCalc_vec.append(errCal_vec3)
    ErrorCalc = o3d.geometry.PointCloud()
    ErrorCalc.points = o3d.utility.Vector3dVector(ErrorCalc_vec)
    error = p2p.compute_rmse(ErrorCalc,tgt_pointCloud,
                             o3d.utility.Vector2iVector(corres_mat))
    return transformation_mat,error

def recursive_combinations(input):
    
    if len(input) == 1:
        return [[a] for a in range(input[0])]
    outcomes = []
    num_of_a_in_current_set= input[0]
    remaining_sets = input[1:]
    for a in range(num_of_a_in_current_set):
        sub_outcomes = recursive_combinations(remaining_sets)
        for sub_outcome in sub_outcomes:
            outcomes.append([a] + sub_outcome)
            
    return outcomes 
  
def registrationWithoutCorrespondence(sourcePoints:np.array, 
                                      targetPoints:np.array):
    
    rowDiff = 0
    
    distMatrixSrc = distance_matrix(sourcePoints, sourcePoints, p=2)
    distMatrixTarg = distance_matrix(targetPoints, targetPoints, p=2)
    
    correspondence ={}
    intersection = {}
    dimension = 3
    registrationError = np.inf
    
    for i in range(len(distMatrixSrc)):
        correspondence[i] = []
        for j in range(len(distMatrixTarg)):
            error = estimatePointToPointError(distMatrixSrc[i], 
                                              distMatrixTarg[j])
            sourceCount = len(distMatrixSrc)-rowDiff
            if np.sum(error < 1) >= dimension and i <= sourceCount:
                for iterI in range(len(distMatrixSrc)):
                    for iterJ in range(len(distMatrixTarg)):
                        elementError = np.abs(distMatrixSrc[i, iterI] 
                                              - distMatrixTarg[j, iterJ])
                        if (elementError < 1) and (i <= len(distMatrixSrc) 
                                                   - rowDiff):
                            if iterI in correspondence.keys():
                                correspondence[iterI].append(iterJ)
                            else:
                                correspondence[iterI] = [iterJ]
        intersection[i] = correspondence
        print(correspondence)
        sourceInd = []
        tgtInd = []
        sourceIndCount = []
        targetIndCount = []
        
        for inp in correspondence.keys():
            sourceInd.append(inp)
            sourceIndCount.append(np.sum(inp == inp))
        
        for inp in correspondence.values():
            tgtInd.append(inp[0] if len(inp)!=0 else [])
            targetIndCount.append(len(inp))
        
        if rowDiff != 0:
            srcIn = np.array(sourcePoints)[:-rowDiff]
        else:
            srcIn = np.array(sourcePoints)
        
        tgtIn = np.array(targetPoints)
        
        if np.sum(sourceIndCount) == np.sum(targetIndCount): 
            val = [x for x in sourceInd]
            val1 = []
            for y in range(len(list(tgtInd))):
                val1.append(list(tgtIn)[y])
            sourcePointsupd = srcIn[val,:]
            targetPointsud = tgtIn[val1,:]
            
            source2target_tf, err = pointBasedRegistration(
                                        sourcePointsupd, 
                                        targetPointsud)
            
            if err < registrationError: 
                registrationError = err
                source2target_tf_min = source2target_tf
        
        else: 
            
            allPossibleOutcomes = recursive_combinations(targetIndCount)
            val = [x for x in sourceInd]
            src_points = srcIn[val,:]
            for tgt_combination, outcome in enumerate(allPossibleOutcomes):
                # tgt_points=[]
                val1 = []
                for i in range(len(correspondence.values())):
                    indexeOfY = list(correspondence.values())[i][outcome[i]]
                    val1.append(indexeOfY)
                tgt_points = tgtIn[val1,:]
                tf_x2y,err = pointBasedRegistration(src_points,tgt_points)
                if err < registrationError:
                    registrationError=err              
                    source2target_tf_min = tf_x2y
        correspondence = {}
    
    return source2target_tf_min, err
            
            
             
        