import rospy
import rosbag
from sensor_msgs.msg import PointCloud2,Image
import tf2_ros 
import geometry_msgs.msg






def main():
 
 while not rospy.is_shutdown():
    rospy.init_node("Pub_Node")
    pub_pc_track = rospy.Publisher("/preprocessor/kinect1/points",PointCloud2, queue_size=1000,latch=True)
    pub_rgba = rospy.Publisher("/preprocessor/kinect1/image", Image, queue_size=1000,latch=True)
    pub_depth = rospy.Publisher("/preprocessor/kinect1/depth", Image, queue_size=1000,latch=True)
    pub_tf=rospy.Publisher("/tf",tf2_ros.TFMessage,queue_size=1000)
    pub_GF=rospy.Publisher("pti_GF",geometry_msgs.msg.PoseArray,queue_size=1000)
    rate = rospy.Rate(50)
    msg_pc=[]
    msg_depth=[]
    msg_im=[]
    msg_tf=[]
    msg_ptiGF=[]
    

    bag = rosbag.Bag('/home/fabio/DeformableDemo/Errori_sim/Bag_simulazione/input_pinch.bag')
    for topic, msg, t in bag.read_messages():
       if topic =='/preprocessor/kinect1/points':
           msg_pc.append(msg)
           # print(type(msg_pc[0]))
           # pub_pc_track.publish(msg)
       elif topic == '/preprocessor/kinect1/depth':
           msg_depth.append(msg)
           # pub_depth.publish(msg)
       elif topic == '/preprocessor/kinect1/image':
           msg_im.append(msg)
           # pub_rgba.publish(msg)
       elif topic == '/tf':
           msg_tf.append(msg)
           # pub_tf.publish(msg)
       elif topic == '/pti_GF':
           msg_ptiGF.append(msg)
           # pub_GF.publish(msg)
    
    bag.close()

    k = 0
    count=0
    for i in range(0,len(msg_pc),10):
    
                    pub_tf.publish(msg_tf[k])
                    pub_tf.publish(msg_tf[k+1])
                    pub_pc_track.publish(msg_pc[i])
                    pub_depth.publish(msg_depth[i])
                    pub_rgba.publish(msg_im[i]) 
                    pub_GF.publish(msg_ptiGF[i])  
                    k=k+20                    
                    rate.sleep()   
                    count+=1
                    print(count)


    rospy.spin()
     
    
    
if __name__ == "__main__":
    main()