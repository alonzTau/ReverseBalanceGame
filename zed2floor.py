import pyzed.sl as sl
import time
import cv2
import numpy as np
import matplotlib.pyplot as mp
import open3d
from scipy.spatial import ConvexHull, transform
from spatialmath import SO3
from sklearn.linear_model import RANSACRegressor
import config_1

def get_rotation(debug = None):
    """operate the zed camera. take a picture and return the normal to the board relative to robot axis.

    Args:
        debug (boolean, optional): allows user to visulize the steps using matplotlib. Defaults to None.

    Returns:
        3X1 nparray: representation of the normal vector
    """
    # Create a ZED camera object
    global zed
    zed = sl.Camera()
    image = sl.Mat()
    point_cloud = sl.Mat()
    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meters as unit (for floor height detection)
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP  # Use a right-handed Y-up coordinate system

    # Open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open the camera")
        zed.close()
        return

    # Enable positional tracking with default parameters
    tracking_params = sl.PositionalTrackingParameters()
    if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable positional tracking")
        zed.close()
        return

    # Detect the floor plane
    zed.grab()
    floor_plane = sl.Plane()
    reset_tracking_floor_frame = sl.Transform()
    if zed.find_floor_plane(floor_plane, reset_tracking_floor_frame) != sl.ERROR_CODE.SUCCESS:
        print("Failed to find the floor plane")
        zed.close()
        return

    # Get the transformation matrix of the floor plane
    floor_transform = reset_tracking_floor_frame

    # Capture images and transform coordinates
    runtime_parameters = sl.RuntimeParameters()
    i = 0
    
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:           
        # Get the camera pose relative to the floor
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        x,y,z,_,_ = get_color(image,point_cloud, "blue") #get all blue pixels
       
        point_stack = np.stack((x,y,z)) #stack the points for easy yse
        rotation_mat =floor_transform.get_rotation_matrix().r
        rotated_points = rotation_mat.dot(point_stack)
        box_points,rotation_matrix,box = calc_OBB_points_rotation(rotated_points) #get oriented boundein box
        if debug:
            fig = mp.figure()
            ax = fig.add_subplot(111, projection='3d')
        
        box_x, box_y, box_z = zip(*(box_points))
        box_x = np.array(box_x)
        box_y = np.array(box_y)
        box_z = np.array(box_z)
        normal, top_indices,bottom = calc_normal_and_top(box,rotation_matrix) #get the normal, ant the top and buttom indices
        closest_points = filter_closest_points(box_x,box_y,box_z,top_indices,normal,rotated_points) #filter for noise
        if debug:
            ax.scatter(closest_points[0],closest_points[1],closest_points[2],label = '80%')
        box_points,rotation_matrix,box = calc_OBB_points_rotation(closest_points) #repeat calculation on filtered points
        normal,top_indices,_ = calc_normal_and_top(box,rotation_matrix) #get the normal, and the top indices
        box_x, box_y, box_z = zip(*(box_points))
        box_x = np.array(box_x)
        box_y = np.array(box_y)
        box_z = np.array(box_z)
        if debug:
            ax.scatter(box_x[top_indices],box_y[top_indices],box_z[top_indices],label = 'box 2')
            ax.legend()
            ax.set_xlabel('$X$', fontsize=20)
            ax.set_ylabel('$Y$')
            ax.set_zlabel('$Z$')
            mp.show()
        i+=1
        if i %5 == 0: 
            if zed.find_floor_plane(floor_plane, reset_tracking_floor_frame) != sl.ERROR_CODE.SUCCESS:
               print("Failed to find the floor plane")
               zed.close()
        #matrix used to match robot orientation in relation to the camera
        R_xyz_to_neg_yxz = np.array([
                                   [0,-1,0],[1,0,0],[0,0,1]
                                ])
        zed.close()
        if debug:
            input()
            x,y,z,_,_ = get_color(image,point_cloud,'red')           
            point_stack_red = np.stack((x,y,z)) 
            # Get camera information (intrinsics) temp code for converting x,y,z to pixel
            cam_info = zed.get_camera_information() 
            fx = cam_info.camera_configuration.calibration_parameters.left_cam.fx  # Focal length in pixels (x axis)
            cy = cam_info.camera_configuration.calibration_parameters.left_cam.cy  # Principal point y
            cx = cam_info.camera_configuration.calibration_parameters.left_cam.cx  # Principal point x
            fy = cam_info.camera_configuration.calibration_parameters.left_cam.fy  # Focal length in pixels (y axis)
            print(f"cx {cx} cy {cy} fy {fy} fx {fx}" )
            array_of_pixels = np.array([])
            print(f"len(point_stack_red) {len(point_stack_red[0])}")
            for  i in range(len(point_stack_red[0])):
                x_of_point =  point_stack_red[0][i]
                y_of_point =  point_stack_red[1][i]
                z_of_point =  point_stack_red[2][i]
                u = int((x_of_point * fx / z_of_point) + cx)
                v = int((y_of_point * fy / z_of_point) + cy)
                np.append(array_of_pixels,(u,v))
            print(f"array_of_pixels {array_of_pixels}")  
            #end temp
        return R_xyz_to_neg_yxz.dot(normal.T) # return rotated normal vector


    # Get the transformation matrix of the floor plane
    floor_transform = reset_tracking_floor_frame


    # Close the camera
    zed.close()

def point_to_plane_distance(point, plane_point, normal):
    """calculate the distance from point to plane

    Args:
        point (ndarray): 3X1 vector representation of the point
        plane_point (ndarray): 3X1 vector representation of a point on the plane
        normal (ndarray): 3X1 vector representation of the normal to the plane

    Returns:
        float: euclidian distance from the plane
    """
    return np.abs(np.dot(normal, point - plane_point))

def rotvec_to_eul(rx, ry, rz):
    """switch from rotvec to euler angles

    Args:
        rx (float): rotation around x
        ry (float): rotation around y
        rz (float): rotation around z

    Returns:
        (float,float,float): eular angles of x,y,z
    """
    v = np.array([rx, ry, rz])
    theta = np.linalg.norm(v)
    v = v / theta
    rot = SO3.AngleAxis(theta, v)
    rz, ry, rx = rot.eul('deg')
    return rx, ry, rz


def get_color(image, point_cloud, color):
    """filter all the pixels in image who match color

    Args:
        image (sl::Mat): image in zed representation (matrix)
        point_cloud (sl::Mat): point cloud in zed representation (matrix)
        color (string): red - filter for red; original - no filter; else - filter for blue

    Returns:
        (list,list,list,list,list): list of x,y,z,x_pixel,y_pixel from pointcloud and image after filtering
    """
    global zed
    
    # Convert the image to an OpenCV format
    cv_image = image.get_data()
    if color == "red":
        # Define the range of blue color in HSV
        lower_blue = np.array([-10, 100, 100])  # RED
        upper_blue = np.array([10, 255, 255])   # RED
    elif color == 'original':
        lower_blue = np.array([0, 0, 0])  # RED
        upper_blue = np.array([255, 255, 255])
    else:
        lower_blue = np.array([90, 120, 120])
        upper_blue = np.array([130,220, 255])


    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    blue_filtered = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    cv2.imshow("Filtered Blue", blue_filtered)

    x_points = []
    y_points = []
    z_point = []
    x_pixel = []
    y_pixel = []
    height = image.get_height()
    width = image.get_width()
    for y in range(300,700):
        for x in range(850,1350):
            # Check if the pixel is blue
            
            if mask[y, x] > 0:
                suc,point3D = point_cloud.get_value(x, y)
                if suc != sl.ERROR_CODE.SUCCESS:
                    continue
                x1 = point3D[0]
                y1 = point3D[1]
                z1 = point3D[2]
                if np.isnan(z1) or z1 is np.isinf(z1):
                    continue
                if np.isnan(x1) is np.nan or np.isinf(x1) :
                    continue
                if np.isnan(y1) is np.nan or np.isinf(y1):
                    continue
                x_points.append(x1)
                y_points.append(y1)
                z_point.append(z1)
                x_pixel.append(x)
                y_pixel.append(y)
    return x_points,y_points,z_point,x_pixel,y_pixel

def calc_OBB_points_rotation(rotated_points):
    """calculate the oriented bounding box from the points

    Args:
        rotated_points (ndarrat): 3XN representation of all of the points

    Returns:
        ((3X8 float), (3X3 ndarray), open3d.geometry.OrientedBoundingBox): the oriented bounding box vertecis, its rotation matrix and the box object
    """
    vect = open3d.utility.Vector3dVector(np.transpose(rotated_points))
    box = open3d.geometry.OrientedBoundingBox.create_from_points(vect)
    box = box.get_minimal_oriented_bounding_box()
    box_points = box.get_box_points()
    rotation_matrix = np.array(box.R)
    return box_points,rotation_matrix,box


def calc_normal_and_top(box,rotation_matrix):
    """calculate the normal from the top plane when axis aligned (is the board plane)

    Args:
        box (open3d.geometry.OrientedBoundingBox)): representation of the OBB
        rotation_matrix (3X3 ndarray): internal rotation matrix

    Returns:
        (3X3 ndarray, list,  list): normal represention and the two indices of box faces he is defined by in the list of box_points
    """
    _,_,box_z = zip(*box.get_box_points())
    box_z = np.array(box_z)
    box.rotate(np.linalg.inv(rotation_matrix))
    box_points = np.array(box.get_box_points())
    z_values = box_points[:, 2]
    # Sort indices based on z-values
    sorted_indices = np.argsort(z_values)

    # Sort points based on sorted indices
    sorted_points = box_points[sorted_indices]
    top_indices = []
    if np.mean(box_z[sorted_indices[4:]]) > np.mean(box_z[sorted_indices[:4]]):
        top_indices = sorted_indices[4:]
        bottom_indices = sorted_indices[:4]

    else:
        top_indices = sorted_indices[:4]
        bottom_indices = sorted_indices[4:]
    mid = np.mean(sorted_points[top_indices],axis=0)
    above = mid +np.array([0,0,mid[2]])

    normal = rotation_matrix.dot(above.T) - rotation_matrix.dot(mid.T)
    normal = -1 *normal if normal[2]<0 else normal
    normal = normal/np.linalg.norm(normal)
    return normal,top_indices,bottom_indices

def filter_closest_points(box_x,box_y,box_z,top_indices,normal,rotated_points, filter = 0.9):
    """filter the points the are closest to the given plane

    Args:
        box_x (ndarray): x values of box points
        box_y (ndarray): y values of box points
        box_z (ndarray): z values of box points
        top_indices (ndarray): 4 indices of the box points we want to filter by
        normal (ndarray): normal to the plane defined by the face
        rotated_points (ndarray): array of the points
        filter (float, optional): filter amount in the range of (0,1). Defaults to 0.9.

    Returns:
        ndarray: filter X len(rotated_points) of the closest points the the plane
    """
    point_on_plane = np.array([box_x[top_indices[0]],box_y[top_indices[0]],box_z[top_indices[0]]])
    distances = np.array([point_to_plane_distance(p, point_on_plane, normal) for p in np.transpose(rotated_points)])
    sorted_indices = np.argsort(distances)
    # Select the closest 80%
    num_points = int(filter * rotated_points.shape[1])
    closest_points_indices = sorted_indices[:num_points]
    closest_points = rotated_points[:,closest_points_indices]
    return closest_points

def two_d_orientation(blue_pixels):
    """2D image of the blue pixels, used in past attempts to find orientation

    Args:
        blue_pixels (ndarray): 2D array of all the relevant pixels
    """
    hull = ConvexHull(blue_pixels)
    #
    ## Get convex hull vertices
    vertices = hull.points[hull.vertices] 
    x,y = zip(*vertices)
    px,py = zip(*blue_pixels)
    mp.scatter(px,py)
    mp.scatter(x,y)
    mp.show()

def plane_of_four_points(points):
    """past attempt using ragnsac to find the plane

    Args:
        points (ndarray): 4x3 array that defines the points

    Returns:
        ransac.estimator_: the estimator that was trained on the points
    """
    ransac = RANSACRegressor()
    
    # Generate combinations of 3 points (for plane fitting)
    
    ransac.fit(points[:, :2], points[:, 2])
    best_plane = ransac.estimator_
    return best_plane


if __name__ == '__main__':
    print(get_rotation(True))
