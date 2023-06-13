from visualization_msgs.msg import MarkerArray 
import numpy as np
import point_cloud_utils as pcu
import std_msgs.msg
from get_point_cloud import *
import geometry_msgs
import pybullet as pb
from geometry_msgs.msg import Pose, PoseArray
import matplotlib.pyplot as plt
import queue as q
from threading import Lock

class calculate_err:
    
   #  lock = Lock()
    
    def __init__(self):
       
       self.sub_ptiGF=rospy.Subscriber("pti_GF",PoseArray,self.callback)
       self.sub_Mark=rospy.Subscriber("cloth_topic",MarkerArray,self.callback1,queue_size=1000)
       self.pub_err_ha_1s=rospy.Publisher("err_topic_ha_1s",std_msgs.msg.Float32,queue_size=1000)
    #    self.pub_err_ha=rospy.Publisher("err_topic_ha",std_msgs.msg.Float32,queue_size=1)
       self.pub_err_ch=rospy.Publisher("err_topic_ch",std_msgs.msg.Float32,queue_size=1000)
    #    self.pub_err_em=rospy.Publisher("err_topic_em",std_msgs.msg.Float32,queue_size=1)
       self.points_GF=[]
       self.coda_GF=q.Queue(1000)
       self.coda_out=q.Queue(1000)
       self.count=0
       self.counterr=0
       self.c = 0
       self.lock=Lock()



    def err(self):
      # with self.lock:
         if (not self.coda_GF.empty()) and (not self.coda_out.empty()):
            ha_1s=pcu.hausdorff_distance(np.array(self.coda_out.get(),order='F'),np.array(self.coda_GF.get(),order='F'))
            

         #  ha=pcu.hausdorff_distance(np.array(self.points_out,order='F'),np.array(self.points_GF_old,order='F'))
            #ch=pcu.chamfer_distance(np.array(self.points_out,order='F'),np.array(self.points_GF_old,order='F'))
            #em=pcu.earth_movers_distance(np.array(self.points_out,order='F'),np.array(self.points_GF_old,order='F'))
            # self.count=self.count+1
            # f=open("Error_em_gravity.txt", "a")
            # f.write("%s\n" % ha_1s[0])
            # f.close()
         #  f=open("Error_chamfer.txt", "a")
         #  f.write("%s\n" % ch)
         #  f.close()

         #  self.pub_err_ha.publish(ha)
         #  self.pub_err_em.publish(em[0])
         #   self.pub_err_ha_1s.publish(ha_1s[0])
            # self.pub_err_ha_1s.publish(ha_1s[0])
            #print(ha_1s[0])
            # print("err "+str(self.count))
            self.counterr+=1
            print(ha_1s)
            


       
    def callback(self,data):
       
      #   with self.lock:
         self.c += 1
         # print("cbP "+str(self.c))
         self.points_GF=[]
         for k in range(0,len(data.poses)):
            self.points_GF.append([data.poses[k].position.x,data.poses[k].position.y,data.poses[k].position.z])
         if self.c>10:
          self.coda_GF.put(self.points_GF)

          
    def callback1(self,data):
        
      #  with self.lock:
        self.count+=1
      #   print("cbM "+str(self.count))
        self.points_out=[]
        for k in range(0,len(data.markers)):
          for j in range(3):
            self.points_out.append([data.markers[k].points[j].x,data.markers[k].points[j].y,data.markers[k].points[j].z])
        
        new_points_out = []
        for elem in self.points_out:
          if elem not in new_points_out:
              new_points_out.append(elem)

        self.points_out=new_points_out

        if self.count>7:
         self.coda_out.put(self.points_out)

        # if not self.points_GF_old:
        #    return
        # else:
        #    self.err(self.points_out,self.points_GF_old)
        


    
def main():

    rospy.init_node("Error_Node")

    ErrorClass=calculate_err()
    
    
    while not rospy.is_shutdown():
#        print(ErrorClass.coda_GF.qsize())
      #  qq=q.Queue(10)
      #  qq.put([2])
      #  qq.put([3])
      #  qq.put([3])
       
      #  print(qq.get())
      #  print(qq.get())
      #  print(qq.get())
          
       
      #rospy.sleep(0.001)
      # if not (ErrorClass.coda_GF.empty() and ErrorClass.coda_out.empty()):
      ErrorClass.err()
      #rospy.sleep(0.005)
      # else:
      #       continue

      
    
       

    
    rospy.spin()
     
    
    
if __name__ == "__main__":
    main()