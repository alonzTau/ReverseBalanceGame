
import rmplab_uri
import numpy as np
from config_1 import *
from control import *
from zed2floor import * 
import scipy.spatial.transform as T
from spatialmath import SO3

#start calibration when the page is loaded
idx = 0
uri = rmplab_uri.RMPLAB_Uri()
uri.teachmode = False
uri.connect(True)
uri.control.moveJ(np.array(init_joints)/180 * np.pi, DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

def calc_offset(row,col):
    """calculate the offset of the location from the board middle

    Args:
        row (int): row number of the desired pawn
        col (int): collum number of the desired pawn

    Returns:
        (float,float,floar): offset from the middle in x,y,z axis
    """
    x_offset = (MID_ROW-row)*ROW_SPACE
    mid = HEXAGON[row]/2
    y_offset = (col-mid)*COL_SPACE
    z_offset = (abs(MID_ROW - row) * 0.0075) + (abs(mid - col) * 0.0075)
    if row%2 == 0:
        z_offset-=0.0025
    #print(f'{x_offset},{y_offset},{z_offset}')
    return x_offset, y_offset, z_offset


def turn(row,col,idx):
    """act one game turn

    Args:
        row (int): row location of pawn to remove
        col (int): collumn location of pawn to remove
        idx (int): number of player
    """
    get_player(0)
    allign_board()
    get_player(0)
    remove_pawn(row,col,idx)

def get_player(idx):
    """move uri to get the pawn to the sideline

    Args:
        idx (int): index of the pawn 
    """
    loc = idx*0.03
    position = pawn_location.copy()
    position[1] -=loc
    uri.control.moveL(position, DEFAULT_SPEED*3, DEFAULT_ACCELERATION*3, False)

def move_above(offset):
    """move above the middle board location as listed in config_1

    Args:
        offset (float): how much above the target you want to be (in meters)
    """
    starting_pos = uri.recieve.getActualTCPPose()
    loc = board_location.copy()
    loc[2] += offset
    starting_pos[2] = loc[2]
    uri.control.moveL(starting_pos, DEFAULT_SPEED, DEFAULT_ACCELERATION, False)
    uri.control.moveL(loc, DEFAULT_SPEED, DEFAULT_ACCELERATION,False)

def put_pawn(pose):
    """
    not used, and not finished
    """
    pass
    rotation_mat = get_rotation()
    starting_pos = uri.recieve.getActualTCPPose()

    R = rotvec_to_rotation_matrix([starting_pos[3],starting_pos[4],starting_pos[5]])
    R_combined = np.dot(rotation_mat, R)
    next_vec = rotation_matrix_to_euler(R_combined)
    starting_pos[3:] = next_vec
    #starting_pos[3:] = (np.array(starting_pos[3:])% np.pi) - np.pi
    print(starting_pos)
    uri.control.moveL(starting_pos,SLOW_SPEED,SLOW_ACCELERATION,False)
    

def move_with_respect_to_board_angle(board_rotation_mat,location):
    """
    not used and not sure if working
    """
    pass
    board_rot = T.Rotation.from_matrix(board_rotation_mat).as_rotvec()

    mid_board_trans = pose_trans(uri,[*board_location[:3],*board_rot],[0,0,0,0,0,0])
    print(f'from {board_rot} to {mid_board_trans}')
    uri.control.moveL(mid_board_trans,SLOW_SPEED,SLOW_ACCELERATION,False)

def remove_pawn(row,col,index):
    """a function that removes a pawn from the board

    Args:
        row (int): row location of pawn to remove
        col (int): collumn location of pawn to remove
        idx (int): number of player
    """
    normal =get_rotation()
    rot_vec = calc_rotvec(normal)
    move_above(0.1)
    move_gripper(uri,0.60,5,5)
    location = list(get_tcp_pose(uri))
    while(location[0]<0): #check and handdle connection exceptions
        uri.disconnect()
        uri.connect(False)
        location = list(get_tcp_pose(uri))
    location[2] -= 0.1
    rot = calculate_adjustments(location,rot_vec,row,col) #adjustments for location
    rot = pose_trans(uri,rot,[0,0,-0.05,0,0,0])
    tcp_movel(uri,*rot) #move above
    rot = pose_trans(uri,rot,[0,0,0.05,0,0,0])
    uri.control.moveL(rot, SLOW_SPEED, SLOW_ACCELERATION,False) #move to pawn
    close_gripper(uri,DEFAULT_GRIPPER_SPEED,DEFAULT_GRIPPER_FORCE)
    rot[2] += 0.1
    tcp_movel(uri,*rot)
    location[2] += 0.1
    tcp_movel(uri,*location)
    get_player(index) #put in the sidelines
    open_gripper(uri,DEFAULT_GRIPPER_SPEED,DEFAULT_GRIPPER_FORCE)

def calc_rotvec(normal):
    """return an apropriate rotvec to rotate the z axis to point to the given normal

    Args:
        normal (nparray): a 3X1 vector

    Returns:
        nparray: a 3X1 rotation vector
    """
    normal = normal/np.linalg.norm(normal)
    theta = np.pi
    if np.abs(normal[1])<=0.01:
        e_z = np.array([0.999,0.0001,0.0001])
        e_z = e_z/np.linalg.norm(e_z)
        y_projection = projection(normal,np.array([1,0,0]))
    elif np.abs(normal[0])<=0.01:
        e_z = np.array([0.0001,0.999,0.0001])
        e_z = e_z/np.linalg.norm(e_z)
        y_projection = projection(normal)
    else:
        e_z = projection(np.array([0,0,1]),normal)
        y_projection = projection(normal,e_z)

    v = 0.5*(e_z + y_projection)
    v = v/np.linalg.norm(v)
    rot_vec = theta*v
    return rot_vec

def calculate_adjustments(location,rot_vec,row,col):
    """calculate the adjustments to line x and y axis withe the orignial aswell as the offset from center

    Args:
        location (list): location you want to calc offset from (usually the center)
        rot_vec (npaarray): a 3X1 rotation vector
        row (int): row of the pawn location
        col (int): collumn of the pawn location

    Returns:
        npaarray: a 6X1 location rotation vector
    """
    rot = np.array(pose_trans(uri,[*location[:3],*rot_vec], [0,0,0,0,0,0]))
    for_projection = np.array(pose_trans(uri,[*location[:3],*rot_vec], [0,1,0,0,0,0]))
    diff = for_projection-rot
    diff = np.array(diff[:2])
    diff = diff/np.linalg.norm(diff)
    y_axis = np.array([0,1])
    angle = np.arccos(np.dot(diff,y_axis))
    temp = pose_trans(uri,[*rot], [0,1,0,0,0,0])
    if temp[0] <rot[0]:
        rot = pose_trans(uri,[*rot], [0,0,0,0,0,angle])
    else:
        rot = pose_trans(uri,[*rot], [0,0,0,0,0,-angle])
    offset = list(calc_offset(row,col))
    rot = pose_trans(uri,[*rot], [*offset,0,0,0])
    return rot

def allign_board():
    """
    allign the board to the correct yaw postion
    """
    normal =get_rotation()
    rot_vec = calc_rotvec(normal)
    move_above(0.15)
    move_gripper(uri,0.6,5,5)
    rot_vec = calculate_adjustments(board_location,rot_vec,MID_ROW,HEXAGON[MID_ROW]/2)
    location = list(get_tcp_pose(uri))
    while(location[0]<0):
        uri.disconnect()
        uri.connect(False)
        location = list(get_tcp_pose(uri))
    
    tcp_movel(uri,*pose_trans(uri,rot_vec,[0,0,-0.04,0,0,0]))
    joints = list(get_q_pose(uri))
    joints[5] = alignment_joints[5]
    q_movej(uri,*(list((np.array(joints))/180 * np.pi)))
    tcp_movel(uri,*rot_vec)
    joints = list(get_q_pose(uri))
    joints[5] = alignment_joints[5]
    q_movej(uri,*(list((np.array(joints))/180 * np.pi)))
    for i in range(22):
        move_gripper(uri,0.7+((i+1)*0.0125),0,0)
        time.sleep(0.1)
    move_gripper(uri,0.7,5,5)
    move_above(0.1)

def play_game(pawn_list):
    """play the game here is where one can implement custom strategy 

    Args:
        pawn_list (list): nX2 pawn location list
    """
    for i,p in enumerate(pawn_list): # right now just iterate and call
        turn(p[0],p[1],i+1)
def main():
    turn(3,6,1)
    turn(0,3,2)
    turn(4,1,3)

