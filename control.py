import rmplab_uri

from utils import *
from config_1 import *


def toggle_connect(uri: rmplab_uri.RMPLAB_Uri, calibrate=True):
    if uri.is_connected():
        uri.disconnect()
    else:
        uri.connect(calibrate)


@requires_connection
def toggle_teachmode(uri: rmplab_uri.RMPLAB_Uri):
    if uri.teachmode:
        uri.control.endTeachMode()
        uri.teachmode = False
    else:
        uri.control.teachMode()
        uri.teachmode = True

@requires_connection
def get_tcp_pose(uri: rmplab_uri.RMPLAB_Uri):
    x, y, z, rx, ry, rz = uri.recieve.getActualTCPPose()
    return x, y, z, rx, ry, rz

@requires_connection
def tcp_movej(uri: rmplab_uri.RMPLAB_Uri, x, y, z, rx, ry, rz):
    uri.control.moveJ_IK([x, y, z, rx, ry, rz], DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

@requires_connection
def tcp_movel(uri: rmplab_uri.RMPLAB_Uri, x, y, z, rx, ry, rz):
    uri.control.moveL([x, y, z, rx, ry, rz], DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

@requires_connection
def get_q_pose(uri: rmplab_uri.RMPLAB_Uri):
    q_pose = np.array(uri.recieve.getActualQ()) * 180 / np.pi
    base, shoulder, elbow, wrist1, wrist2, wrist3 = q_pose
    return base, shoulder, elbow, wrist1, wrist2, wrist3

@requires_connection
def q_movej(uri: rmplab_uri.RMPLAB_Uri, base, shoulder, elbow, wrist1, wrist2, wrist3):
    uri.control.moveJ([base, shoulder, elbow, wrist1, wrist2, wrist3], DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

@requires_connection
def q_movel(uri: rmplab_uri.RMPLAB_Uri, base, shoulder, elbow, wrist1, wrist2, wrist3):
    uri.control.moveL_FK([base, shoulder, elbow, wrist1, wrist2, wrist3], DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

@requires_connection
def get_gripper_pos(uri: rmplab_uri.RMPLAB_Uri):
    return uri.gripper.get_current_position()

@requires_connection
def move_gripper(uri:rmplab_uri.RMPLAB_Uri, pos, speed, force):
    uri.gripper.move_and_wait_for_pos(pos, speed, force)

@requires_connection
def close_gripper(uri:rmplab_uri.RMPLAB_Uri, speed, force):
    uri.gripper.close(speed, force)

@requires_connection
def open_gripper(uri:rmplab_uri.RMPLAB_Uri, speed, force):
    uri.gripper.open(speed, force)

@requires_connection
def pose_trans(uri:rmplab_uri.RMPLAB_Uri, p_from, p_from_to):
    '''
    pose transformtaion to move with respect to custom feature space.
    rotate and translate p_from_to by the parameters of p_from_to. 
    '''
    return uri.control.poseTrans(p_from,p_from_to)

@requires_connection
def move_gripper(uri,closed_prectenge: float, speed: int, force: int):
    '''
    move gripper from current position to new position (not open and closed all the way)
    '''
    position = int(closed_prectenge*(uri.gripper.get_closed_position()-uri.gripper.get_open_position()))
    uri.gripper.move_and_wait_for_pos(position,speed,force)