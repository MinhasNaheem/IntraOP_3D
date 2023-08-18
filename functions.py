import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance_matrix
import matplotlib.pyplot as plt
import open3d as o3d
import plotly
import plotly.graph_objects as go

# rotate a given set of points
def rotate_vec(r,p_vec):
    rotated_vec = []
    for i in range (len(p_vec)):
        rot_vec =r.apply(p_vec[i])
        rotated_vec.append(rot_vec)  
    return np.array(rotated_vec)

# check size shape and size of A and B to estimate the stray points
def check_size(src,tgt):
    if src.shape == tgt.shape:
        rows_diff = 0

    else:
        if src.size > tgt.size:
            small_matrix = tgt
            big_matrix = src
        else:
            small_matrix = src
            big_matrix = tgt
            
        rows_diff = big_matrix.shape[0] - small_matrix.shape[0]

        if rows_diff > 0:
            zeros = np.zeros((rows_diff, small_matrix.shape[1]))
            small_matrix = np.concatenate((small_matrix, zeros), axis=0)
        
        if src.size > tgt.size:
            tgt = small_matrix
        else:
            src = small_matrix

    return src, tgt, rows_diff


#  estimate point to point error
def p2p_error(src,tgt):
    error = []
    for i in range(len(src)):
        min_error = float("inf")
        relative_difference = np.abs(tgt - src[i])  
        for k in range(len(relative_difference)):
            if relative_difference[k] < min_error:
                min_error = relative_difference[k]
        error.append(min_error)
    return np.array(error)

# determine the src and tgt combinations if there isn't one-to-one correspondence
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

# perform registration of 2 points
def Registration(src,tgt):
    src_pointCloud = o3d.geometry.PointCloud()
    src_pointCloud.points = o3d.utility.Vector3dVector(src)
    tgt_pointCloud = o3d.geometry.PointCloud()
    tgt_pointCloud.points = o3d.utility.Vector3dVector(tgt)
    ind = np.arange(len(src))
    cor = np.hstack((ind,ind))
    
    corres_mat = np.vstack((ind,ind)).transpose()
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    transformation_mat = p2p.compute_transformation(src_pointCloud, tgt_pointCloud,o3d.utility.Vector2iVector(corres_mat))
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
    error = p2p.compute_rmse(ErrorCalc,tgt_pointCloud,o3d.utility.Vector2iVector(corres_mat))
    return transformation_mat,error

# plotting the fiducials
def plot3d(src,tgt):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    tgt= np.array(tgt)
    ax.scatter(src[:,0], src[:,1], src[:,2], c='blue', label='X')
    ax.scatter(tgt[:,0], tgt[:,1], tgt[:,2], c='red', label='Y')

    # Connect corresponding points
    for ind in range(len(tgt)):
        x_connect = src[ind]
        y_connect = tgt[ind]
        ax.plot([x_connect[0], y_connect[0]], [x_connect[1], y_connect[1]], [x_connect[2], y_connect[2]], c='green', linestyle='dashed')
        ax.text(x_connect[0], x_connect[1], x_connect[2], str(src[ind]), color='black', fontsize=8, horizontalalignment='left')
        ax.text(y_connect[0], y_connect[1], y_connect[2], str(tgt[ind]), color='black', fontsize=8, horizontalalignment='left')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Corresponding X and Y Points')
    ax.legend()
    plt.grid(True)
    plt.show()

def plot_fids(fids):
    fids=np.vstack((fids,[0, 0, 0]))
    fiducials = go.Scatter3d(
        x=fids[:,0], y=fids[:,1], z=fids[:,2],
        marker=dict(
            size=4,
            colorscale='Viridis',
        ),
        line=dict(
            color='darkblue',
            width=2
        )
    )
    axes   = go.Scatter3d ( x = [0, 0,   0  , 100, 0, 0  ],
                            y = [0, 100, 0  , 0,   0, 0  ], 
                            z = [0, 0,   0  , 0,   0, 100], 
                            marker = dict(  size = 1,
                                            color = "rgb(84,48,5)"),
                                            line = dict( color = "rgb(84,48,5)",
                                            width = 6)
                            )
    data = [fiducials,axes]
    name = 'default'

    # Default parameters which are used when `layout.scene.camera` is not provided
    camera = dict(
                    up=dict(x=-1, y=0, z=0),
                    center=dict(x=0, y=0, z=0),
                    eye = dict(x=0, y=0, z=1.25)
                 )

    fig = go.Figure(data=data)

    fig.update_layout(scene_camera=camera, title=name)

    fig.update_layout(
        scene = dict(
                        xaxis = dict(nticks=4, range=[-3000,3000],),
                        yaxis = dict(nticks=4, range=[-3000,3000],),
                        zaxis = dict(nticks=4, range=[-3000,3000],),
                     ),
                    width=700,
                    margin=dict(r=20, l=10, b=10, t=10))

    return fig

# compute correspondence between two sets of points and perform registration
def correspondence(src,tgt,rows_diff=0,plot=True):
    
    # check for stray points
    src,tgt,rows_diff = check_size(src,tgt) 

    # intermarker distance calculation
    dist_src = distance_matrix(src,src,p=2)
    dist_tgt = distance_matrix(tgt,tgt,p=2)
    
    corres = {}
    intersec_ip = {}
    iterr_i = 0
    dimensions = 3
    reg_error = np.inf

    for i in range(len(dist_src)):
        corres[i] = []
        for j in range(len(dist_tgt)):
            err = p2p_error(dist_src[i],dist_tgt[j])
            src_count = len(dist_src)-rows_diff
            if np.sum(err<1) >= dimensions and i <= src_count:
                for iterr_i in range(len(dist_src)):
                    for iterr_j in range(len(dist_tgt)):
                        element_err = np.abs(dist_src[i, iterr_i] - dist_tgt[j, iterr_j])
                        if element_err < 1 and i <= len(dist_src)-rows_diff :
                            if iterr_i in corres.keys():
                                corres[iterr_i].append(iterr_j)
                            else:
                                corres[iterr_i] = [iterr_j]
        
        intersec_ip[i] = corres
        print(corres)

        #backtrack src and tgt points
        src_ind = []
        tgt_ind = []
        src_ind_count = []
        tgt_ind_count = []

        for inp in corres.keys():
            src_ind.append(inp)
            src_ind_count.append(np.sum(inp==inp))

        for inp in corres.values():
            tgt_ind.append(inp[0] if len(inp)!=0 else [])
            tgt_ind_count.append(len(inp))

        if rows_diff != 0 :
            src_in = np.array(src)[:-rows_diff]
        else:
            src_in = np.array(src)
        
        tgt_in = np.array(tgt)

        if np.sum(src_ind_count)==np.sum(tgt_ind_count):
            val = [x for x in src_ind]
            val1 = []
            for y in range(len(list(tgt_ind))):
                val1.append(list(tgt_ind)[y])
            src_points = src_in[val,:]
            tgt_points = tgt_in[val1,:]
            tf_x2y,err = Registration(src_points,tgt_points)
            if err < reg_error:
                reg_error=err
                tf_x2y_minErr = tf_x2y

        else:
            all_possible_outcomes = recursive_combinations(tgt_ind_count)
            val = [x for x in src_ind]
            src_points = src_in[val,:]
            for tgt_combination, outcome in enumerate(all_possible_outcomes):
                # tgt_points=[]
                val1 = []
                for i in range(len(corres.values())):
                    indexeOfY = list(corres.values())[i][outcome[i]]
                    val1.append(indexeOfY)
                tgt_points = tgt[val1,:]
                tf_x2y,err = Registration(src_points,tgt_points)
                if err < reg_error:
                    reg_error=err              
                    tf_x2y_minErr = tf_x2y

        corres = {}

    print("Corresponding indices of SRC and TGT")
    print("SRC Index \n", val)
    print("TGT Index \n", val1)
    print("Source Points \n", src_points)
    print("Target Points \n", np.asarray(tgt_points))
    print("Registration Error \n", reg_error)
    print("Transformation src2tgt \n", tf_x2y_minErr)
    
    # if plot == True and len(tf_x2y_minErr)!=0:
    #     plot_points = plot3d(src_points,tgt_points)

    #     tgt_pts_homogenous = np.column_stack((np.matrix(tgt_points), np.ones((len(tgt_points), 1))))
    #     src_transformed_pts = tgt_pts_homogenous@tf_x2y_minErr
    #     src_transformed_pts[:,:3]

    #     fig1 = plt.figure()
    #     ax1 = fig1.add_subplot(111, projection='3d')
    #     ax1.scatter(src_points[:,0], src_points[:,1], src_points[:,2], c='green', label='X_source')
    #     ax1.scatter(src_transformed_pts[:,0], src_transformed_pts[:,1], src_transformed_pts[:,2], c='yellow', label='X_transformed')
    #     plt.show()

    return intersec_ip, tf_x2y_minErr, reg_error
