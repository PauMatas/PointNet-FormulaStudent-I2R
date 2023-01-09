from typing import List, Tuple


Points = List[Tuple[float, float, float]]

import numpy as np
 
def quaternion_rotation_matrix(position):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract position
    x = position[0]
    y = position[1]
    z = position[2]

    # Extract the values from Q
    qx = position[3]
    qy = position[4]
    qz = position[5]
    qw = position[6]
    
    r11 = 1 - 2*qy*qy - 2*qz*qz
    r12 = 2*qx*qy - 2*qz*qw
    r13 = 2*qx*qz + 2*qy*qw
    r21 = 2*qx*qy + 2*qz*qw
    r22 = 1 - 2*qx*qx - 2*qz*qz
    r23 = 2*qy*qz - 2*qx*qw
    r31 = 2*qx*qz - 2*qy*qw
    r32 = 2*qy*qz + 2*qx*qw
    r33 = 1 - 2*qx*qx - 2*qy*qy

     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r11, r12, r13, x],
                           [r21, r22, r23, y],
                           [r31, r32, r33, z]])


                            
    return rot_matrix

def transform_frame(frame_lidar, position_cones, position) -> Tuple[Points, Points]:
    transform_matrix = quaternion_rotation_matrix(position)
    raise NotImplementedError