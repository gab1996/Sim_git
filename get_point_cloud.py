""" Get point cloud from pybullet camera


"""

import pybullet as pb
import numpy as np

import rospy
import sensor_msgs.msg
import std_msgs.msg
import math
def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.msg.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    # fields = [sensor_msgs.msg.PointField(
    #     name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
    #     for i, n in enumerate('xyzrgba')]

    fields = [sensor_msgs.msg.PointField('x', 0, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('y', 4, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('z', 8, sensor_msgs.msg.PointField.FLOAT32, 1),
                  sensor_msgs.msg.PointField('rgb', 12, sensor_msgs.msg.PointField.FLOAT32, 3),
                  sensor_msgs.msg.PointField('a', 24, sensor_msgs.msg.PointField.FLOAT32, 1)]

    header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.msg.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )

def get_point_cloud(width, height, view_matrix, proj_matrix, unique_id: int = None, full: bool = False, frame : str = 'world'):
    """Get point cloud from pybullet cameras. Optionally specify target object unique ID.

    Based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer
    """

    # get a depth image
    # "infinite" depths will have a value close to 1
    image_arr = pb.getCameraImage(
        width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    rgba: np.ndarray = image_arr[2]
    # print(rgba.shape)
   # print(rgba)
    depth: np.ndarray = image_arr[3]

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    # print(proj_matrix)
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    # print("--------")
    # print(proj_matrix)
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))
    # print(proj_matrix)
    # print(np.linalg.inv(view_matrix))
    # print(proj_matrix)
    # print("-------------")
    # print(view_matrix)
    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)
    
    r, g, b, a = rgba[:, :, 0].reshape(-1), rgba[:, :, 1].reshape(-1), rgba[:, :, 2].reshape(-1), rgba[:, :, 3].reshape(-1)

    

    pixels: np.ndarray = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    if unique_id is not None:
        mask = image_arr[4]
        m = mask.reshape(-1)
        pixels = pixels[m == unique_id]
        z = z[m == unique_id]
        r = r[m == unique_id]
        g = g[m == unique_id]
        b = b[m == unique_id]
        a = a[m == unique_id]
    pixels = pixels[z < 0.99]
    pixels[:, 2] = 2 * pixels[:, 2] - 1
    roll=-math.pi   
    rollMatrix = np.matrix([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
    ])
    
    # turn pixels to world coordinates
    points_GF = np.matmul(tran_pix_world, pixels.T).T
   
    points = np.matmul(np.linalg.inv(proj_matrix), pixels.T).T
    
    
    # for i in range(len(points)):

        


    #points=np.dot(T_m,points)
    ##########
    # O,T=np.linalg.qr(np.linalg.inv(proj_matrix))D
    # R=np.linalg.inv(O)
    # K=np.linalg.inv(T)
    # t=np.dot(np.linal.inv(K),pto)
    ########
    

    #A=np.array([[[0.02379351,0.98803162,0.15240531,0.6],[-0.15240531,0.15425145,-0.97620649,0.1 ],[-0.98803162,0.,0.15425145,0.6],[0,0,0,1]]])
    #points = np.matmul(np.linalg.inv(proj_matrix), pixels.T).T
    # O,T=np.linalg.qr(np.linalg.inv(proj_matrix))
    # R=np.linalg.inv(O)
    # #print(R)
    # K=np.linalg.inv(T)
    # A=np.matrix([[R.item(0,0),R.item(0,1),R.item(0,2),0.6],[R.item(1,0),R.item(1,1),R.item(1,2),0.1],[R.item(2,0),R.item(2,1),R.item(2,2),0.6],[0,0,0,1]])
    # print(A)
    # points = np.matmul(A, pixels.T).T
    # print(proj_matrix)
    # print("-----------------")
    # print(view_matrix)
    points[:, :3] /= points[:, 3: 4]
    points = points[:, :3]
    
    
    points_GF[:, :3] /= points_GF[:, 3: 4]
    points_GF = points_GF[:, :3]

    points_GF=np.array(points_GF,order='F')
    

    for i in range(0,len(points[:,1])):
          pt=np.array([points[i,0],points[i,1],points[i,2]])
          tmp=np.array(np.dot((rollMatrix),pt))
          points[i,:]=tmp


    #print(points)

    #print(points)
    if full:
        depth = (depth-np.min(depth)) / \
            (np.max(depth)-np.min(depth))*255
        # print(np.min(depth))
        # print(np.max(depth))
        # # depth=depth.astype(np.uint8)
        # print(depth.shape)
        # print(np.zeros(depth.shape).shape)
        # depth = np.stack([np.zeros(depth.shape), np.zeros(
        #     depth.shape), depth, np.ones(depth.shape)], axis=2)
        depth = depth.astype(np.uint8)

        pc2 = point_cloud(np.stack([points[:,0], points[:,1], points[:,2], r/255.0, g/255.0, b/255.0, a/255.0], axis=1), frame)
       
        return points, rgba, depth, pc2, points_GF

    return points
