import tkinter as tk

import numpy as np
from spatialmath import SO3
import math
import rmplab_uri

def requires_connection(f):
    def f_(uri: rmplab_uri.RMPLAB_Uri, *args, **kwargs):
        if not uri.is_connected():
            print("Uri is not connected! Abort.")
            return
        return f(uri, *args, **kwargs)
    return f_
        
def rotvec_to_rotation_matrix(rotvec):
    angle = np.linalg.norm(rotvec)
    axis = rotvec / angle if angle != 0 else np.zeros(3)
    
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
    return R

def rotation_matrix_to_euler(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])

def rotvec_to_eul(rx, ry, rz):
    v = np.array([rx, ry, rz])
    theta = np.linalg.norm(v)
    v = v / theta
    rot = SO3.AngleAxis(theta, v)
    rz, ry, rx = rot.eul('deg')
    return rx, ry, rz

def eul_to_rotvec(rx, ry, rz):
    rot = SO3.Eul(rz, ry, rx, unit='deg')
    theta, v = rot.angvec()
    rx, ry, rz = theta * v
    return rx, ry, rz

def set_text(e: tk.Entry, text):
    e.delete(0, tk.END)
    e.insert(0, text)
    return


def projection(normal: np.array, to_project = None):
    '''
    function to calculate projection on a plane induced by normal
    @param normal np.array to represent the plane normal we want to project on
    @paran to_project vector to project. if left None will project the (0,1,0) vector
    @return normalized projection
    '''
    if to_project is None:
        to_project = np.array([0,1,0])
    to_project = to_project/np.linalg.norm(to_project)
    projection = to_project - np.dot(to_project,normal)/(np.dot(normal,normal)) * normal
    norm = np.linalg.norm(projection)
    if(norm != 0):
        projection = projection/norm
    return projection

if __name__ == "__main__":
    print(rotvec_to_eul(2.2, 2.2, 0))

    rx, ry, rz = 45, 90, -45
    print(eul_to_rotvec(rx, ry, rz))
