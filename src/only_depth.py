#!/usr/bin/env python3

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from waffle.msg import Tupla
import rospy
import numpy as np
import cv2
import threading
from time import sleep
import sys

class Only(threading.Thread):

    def __init__(self) -> None:

        self.w_depth=320
        self.h_depth=240
        np.set_printoptions(threshold=sys.maxsize)
        threading.Thread.__init__(self)
        self.image=np.zeros((480,640),np.uint8)
        self.ang_vel=0
        self.pub_vel=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.pub_tupla=rospy.Publisher("/control_value",Tupla,queue_size=10)
        self.vel=Twist()
        self.tupla=Tupla()
        rospy.Subscriber("/camera/depth/image_raw",Image,self.camera_cb)
        self.bridge=CvBridge()

    def move(self,linear_x,angular_theta):

        self.vel.linear.x=linear_x
        self.vel.angular.z=angular_theta
        self.pub_vel.publish(self.vel)

    def com(self,flag,controller_data):
        
        self.tupla.flag=flag
        self.tupla.control=controller_data
        self.pub_tupla.publish(self.tupla)

    def camera_cb(self,msg):

        self.image=cv2.resize(self.bridge.imgmsg_to_cv2(msg,"passthrough"),(self.w_depth,self.h_depth))
        # print(self.image)
        # self.image=self.image*0.001

def find_window(frame):

    threshold=0.06
    r,col=frame.shape

    frame=np.where(frame>threshold,1.0,0)
    return frame

if __name__=="__main__":

    rospy.init_node("only_depth")

    area_threshold=2000
    o=Only()
    print("Initializing...")
    sleep(5)
    while not rospy.is_shutdown():
 
        image=np.array(o.image,dtype=np.float32)
        cv2.normalize(image,image,0,1,cv2.NORM_MINMAX)
        img_view=image.copy()
        image=find_window(image)
        img_cast=image.astype("uint8")

        contours,hierarchy=cv2.findContours(img_cast,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        area=np.zeros(len(contours))
        print(area)
        for ind in range(len(contours)):
            cnt=contours[ind]
            area[ind]=cv2.contourArea(cnt)
            
        index_max_rect=np.argmax(area)
        cnt2=contours[index_max_rect]

        if(area[index_max_rect]>area_threshold):
            xx,yy,ww,hh=cv2.boundingRect(cnt2)
            cv2.rectangle(img_view,(xx,yy),(xx+ww,yy+hh),(0,255,0),2)
            o.com(0,int(xx+(ww/2)-((o.w_depth/2)-1)))
            print(int(xx+(ww/2)-((o.w_depth/2)-1)))

        cv2.imshow("depth view",image)
        cv2.imshow("bounded_image",img_view)

        k=cv2.waitKey(1) & 0xFF
        if k==27:
            o.move(0,0)
            break

    cv2.destroyAllWindows()
    rospy.signal_shutdown("Closing the program...")
