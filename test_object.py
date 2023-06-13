""" Bending simulator of our prismatic test object

Simulate bending of a prismatic soft body partially lying on a table,
also generate synthetic point cloud. 
"""

import tf2_ros
import geometry_msgs.msg
import pybullet as pb
import pybullet_data
import numpy as np
from get_point_cloud import *
import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image
import sensor_msgs.msg as sensor_msgs
import transformations as tfm
from std_msgs.msg import Header
import math
import point_cloud_utils as pcu
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct

from threading import Lock
from geometry_msgs.msg import Pose, PoseArray

# record True/False of the synthetic point cloud
LOGPTC = True

# record True/False the simulated physics as MP4
LOGVIDEO = False

LOGRGB = True

LOGDEPTH = True

PINCH_GRASP= False

LATERAL_GRASP = False

LATERAL_POINT = True

LOGERR= True


# def ros_to_pcl(ros_cloud):
#     """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

#         Args:
#             ros_cloud (PointCloud2): ROS PointCloud2 message

#         Returns:
#             pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
#     """
#     points_list = []

#     for data in pc2.read_points(ros_cloud, skip_nans=True):
#         points_list.append([data[0], data[1], data[2], data[3]])

#     pcl_data = pcl.PointCloud_PointXYZRGB()
#     pcl_data.from_list(points_list)
#    return


# def get_mesh_data(sim, deform_id):
#     """Returns num mesh vertices and vertex positions."""
#     kwargs = {}
#     if hasattr(pb, 'MESH_DATA_SIMULATION_MESH'):
#         kwargs['flags'] = pb.MESH_DATA_SIMULATION_MESH
#     num_verts, mesh_vert_positions = sim.getMeshData(deform_id, **kwargs)
#     return num_verts, mesh_vert_positions

# def conversion_pc(ros_point_cloud):
#                 xyz = np.array([[0,0,0]])
#                 rgb = np.array([[0,0,0]])
#                 #self.lock.acquire()
#                 gen = pc2.read_points(ros_point_cloud, skip_nans=True)
#                 int_data = list(gen)

#                 for x in int_data:
#                     test = x[3] 
#                     # cast float32 to int so that bitwise operations are possible
#                     s = struct.pack('>f' ,test)
#                     i = struct.unpack('>l',s)[0]
#                     # you can get back the float value by the inverse operations
#                     pack = ctypes.c_uint32(i).value
#                     r = (pack & 0x00FF0000)>> 16
#                     g = (pack & 0x0000FF00)>> 8
#                     b = (pack & 0x000000FF)
#                     # prints r,g,b values in the 0-255 range
#                                 # x,y,z can be retrieved from the x[0],x[1],x[2]
#                     xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
#                     rgb = np.append(rgb,[[r,g,b]], axis = 0)
#                     print(xyz)



class simple_class:
  
  lock = Lock()
  def __init__(self):

    self.pub=rospy.Publisher("pti_GF",geometry_msgs.msg.PoseArray,queue_size=1000)
    self.points_GF=None
    self.count=0


  def update_GF(self,points_GF):
     with self.lock:
        self.points_GF=points_GF
        pose_array = PoseArray()
        for i in range(len(self.points_GF)):
            pose = Pose() # create a new Pose message
            pose.position.x, pose.position.y, pose.position.z = self.points_GF[i,0],self.points_GF[i,1],self.points_GF[i,2]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0,0,0,1
            pose_array.poses.append(pose) # add the new Pose object to the PoseArray list
        
        self.pub.publish(pose_array)
        # self.count+=1
        # print(self.count)
        

     

def main():

    rospy.init_node("create_cloud")
    pub_pc_track = rospy.Publisher("/preprocessor/kinect1/points",
                          PointCloud2, queue_size=1000)
    pub_rgba = rospy.Publisher("/preprocessor/kinect1/image", Image, queue_size=1000)
    pub_depth = rospy.Publisher("/preprocessor/kinect1/depth", Image, queue_size=1000)
    a=simple_class()
    SendTf = tf2_ros.TransformBroadcaster()
    rate=rospy.Rate(50)


    # create a physics client
    physicsClient = pb.connect(pb.GUI)

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())

    # init simulation environment
    pb.resetDebugVisualizerCamera(0.1, 30, -30, [0.6, 0.1, 0.6])

    pb.resetSimulation(pb.RESET_USE_DEFORMABLE_WORLD)
    
    pb.setGravity(0, 0, -10)
    t_step=0.002
    #t_step=0.005
    pb.setTimeStep(t_step)

    # add plane with a table (and object for pull up the insole tip)
    planeId = pb.loadURDF("data/plane_with_box.urdf", [0, 0, 0])
    # load our prismatic soft body
    objId = pb.loadSoftBody("data/sola_bene_m_pb.vtk",
                            simFileName="data/sola_bene_m_pb.vtk",
                            basePosition=[0.3, 0.25, 0.5],
                            mass=0.1,
                            useMassSpring=1,
                            useBendingSprings=1,
                            collisionMargin=0.001,
                            frictionCoeff=0.4,
                            springElasticStiffness=30,
                            springDampingStiffness=50,
                            springBendingStiffness=30,
                            # springElasticStiffness=10,
                            # springDampingStiffness=0.1,
                            # springBendingStiffness=10,
                            useFaceContact=1,
                            repulsionStiffness=10,
                            useSelfCollision=0)
    
    roll = 0 #x
    pitch = -30*math.pi/180#y
    yaw = 30.0*math.pi/180.0#z
    
    # yawMatrix = np.matrix([
    # [math.cos(yaw), -math.sin(yaw), 0],

    # [math.sin(yaw), math.cos(yaw), 0],
    # [0, 0, 1]
    # ])
    
    # pitchMatrix = np.matrix([
    # [math.cos(pitch), 0, math.sin(pitch)],
    # [0, 1, 0],
    # [-math.sin(pitch), 0, math.cos(pitch)]
    # ])

    # rollMatrix = np.matrix([
    # [1, 0, 0],
    # [0, math.cos(roll), -math.sin(roll)],
    # [0, math.sin(roll), math.cos(roll)]
    # ])
    
    # R = yawMatrix * pitchMatrix * rollMatrix

    QQ=tfm.quaternion_about_axis(math.pi/2,[1,0,0])
    QQ=tfm.quaternion_multiply(QQ,tfm.quaternion_about_axis(yaw,[0,1,0]))
    QQ=tfm.quaternion_multiply(QQ,tfm.quaternion_about_axis(pitch,[1,0,0]))
    QQ1=tfm.quaternion_about_axis(-math.pi,[1,0,0])
    PLP=tfm.quaternion_matrix(QQ)
    z=PLP[:3,2]
    z/=np.linalg.norm(z)
    
    width = 640
    height = 480
    tr=rospy.Time.now()
    if LOGPTC:
        # retrieve debug camera parameters
        debug_camera = pb.getDebugVisualizerCamera()
        
        # choose resolutionrospy.Time.now()

        # get and save ptc at start time
        points, rgba, depth, pc_tracker, points_GF = get_point_cloud(
            width, height, debug_camera[2], debug_camera[3], objId, True, "kinect1_rgb_optical_frame_sim")
        
        # a=simple_class(points_GF)
        # a.update_GF(points_GF)

        
        t = np.array([0.6,0.1,0.6])+0.1*z

        # M=np.matmul(M,tfm.translation_matrix((t[0],t[1],t[2])))
        #M=np.linalg.inv(M)
        #M=M.T
        #M=tfm.compose_matrix(angles=tfm.euler_from_quaternion(QQ),translate=[t[0],t[1],t[2]])  
        
        # points_GF=np.ndarray(shape=(len(points[:,1]*3),4))
        # for i in range(0,len(points[:,1])):
        #   pt=np.array([points[i,0],points[i,1],points[i,2],1])
        #   tmp=np.array(np.dot((M),pt))
        #   points_GF[i,:]=tmp

        # points_GF=np.array(points_GF[:,:3])
        
        #print(points_GF)


        part = "data/points/points_{part:03d}.csv"
        np.savetxt(part.format(part=0), points, delimiter=",")
        
        

        CamToWorld = geometry_msgs.msg.TransformStamped()
        CamToWorld.transform.translation.x=t[0]
        CamToWorld.transform.translation.y=t[1]
        CamToWorld.transform.translation.z=t[2]
        CamToWorld.transform.rotation.x=QQ[1]
        CamToWorld.transform.rotation.y=QQ[2]
        CamToWorld.transform.rotation.z=QQ[3]
        CamToWorld.transform.rotation.w=QQ[0]
        CamToWorld.header.frame_id = "ground"
        CamToWorld.header.stamp = tr
        CamToWorld.child_frame_id = "kinect1_rgb_optical_frame_sim"


        CamTrackerToCam=geometry_msgs.msg.TransformStamped()
        CamTrackerToCam.header.frame_id="kinect1_rgb_optical_frame_sim"
        CamTrackerToCam.header.stamp=CamToWorld.header.stamp
        CamTrackerToCam.child_frame_id="kinect1_rgb_optical_frame"
        CamTrackerToCam.transform.translation.x=0
        CamTrackerToCam.transform.translation.y=0
        CamTrackerToCam.transform.translation.z=0
        CamTrackerToCam.transform.rotation.x=QQ1[1]
        CamTrackerToCam.transform.rotation.y=QQ1[2]
        CamTrackerToCam.transform.rotation.z=QQ1[3]
        CamTrackerToCam.transform.rotation.w=QQ1[0]
        
        # SendTf.sendTransform(CamToWorld)  
        # SendTf.sendTransform(CamTrackerToCam)       

        pc_tracker.header.stamp=tr
        pc_tracker.header.frame_id="kinect1_rgb_optical_frame"
        # pub_pc_track.publish(pc_tracker)
        


    if LOGRGB:
        im = Image()
        im.header.frame_id = "kinect1_rgb_optical_frame"
        im.header.stamp=tr
        im.is_bigendian = False
        im.data = np.ndarray.tolist(np.reshape(rgba, 4*width*height))
        im.height = height
        im.width = width
        im.step = width*4
        #im.encoding = '8UC4'
        im.encoding = 'rgba8'
        # pub_rgba.publish(im)
        
    if LOGDEPTH:
        depthh = Image()
        depthh.header.frame_id = "kinect1_rgb_optical_frame"
        depthh.header.stamp=tr
        depthh.is_bigendian = False
        depthh.data = np.ndarray.tolist(np.reshape(depth, width*height))
        depthh.height = height
        depthh.width = width
        depthh.step = width
        depthh.encoding = 'mono8'
        # pub_depth.publish(depthh)

    if LOGVIDEO:
        pb.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4,
                             fileName="data/test_insole.mp4")


    #print(pb.getNumJoints(planeId))
    c = 0


    # perform some simulation steps
    while pb.isConnected() and c < 1000:

        
        # input()
        pb.stepSimulation()
        pb.setGravity(0, 0, -10)

        if c>1:
          if PINCH_GRASP:
              index_pinch=[3,4,5,6,170,172]
              for i in range(len(index_pinch)):
                pb.createSoftBodyAnchor(objId ,index_pinch[i],planeId,1) 
                pb.setJointMotorControl2(planeId,1,pb.POSITION_CONTROL,0.1,0,maxVelocity=0.05)
          if LATERAL_GRASP:
              index_lateral=[3,5,9,10,11,104,105,119,120,122,169,170,172,200]
              for i in range(len(index_lateral)):
                pb.createSoftBodyAnchor(objId ,index_lateral[i],planeId,1) 
                pb.setJointMotorControl2(planeId,1,pb.POSITION_CONTROL,0.1,0,maxVelocity=0.05)
          if LATERAL_POINT:
              index_point=[10,115,119,120,122,200,201,202]
              for i in range(len(index_point)):
                pb.createSoftBodyAnchor(objId ,index_point[i],planeId,1) 
                pb.setJointMotorControl2(planeId,1,pb.POSITION_CONTROL,0.1,0,maxVelocity=0.05)


        if LOGPTC:


                points, rgba, depth, pc_tracker, points_GF = get_point_cloud(
                width, height, debug_camera[2], debug_camera[3], objId, True, "kinect1_rgb_optical_frame")
                np.savetxt(part.format(part=c), points, delimiter=",")

                
                pc_tracker.header.stamp=tr
                
                CamToWorld.header.stamp = tr
                CamTrackerToCam.header.stamp = tr
                im.data = np.ndarray.tolist(np.reshape(rgba, 4*width*height))
                im.header=pc_tracker.header
                depthh.header=pc_tracker.header
                depthh.data = np.ndarray.tolist(np.reshape(depth, width*height))

                a.update_GF(points_GF)
                SendTf.sendTransform(CamToWorld)  
                SendTf.sendTransform(CamTrackerToCam)
                pub_pc_track.publish(pc_tracker)
                pub_rgba.publish(im)
                pub_depth.publish(depthh)
                rate.sleep()


        else:
            # update cameras (if get_point_cloud wasn't called)
            pb.getCameraImage(width, height)
        c += 1
        tr=tr+rospy.Duration(t_step)

    if LOGVIDEO:
        pb.stopStateLogging(pb.STATE_LOGGING_VIDEO_MP4)

    


if __name__ == "__main__":
    main()
