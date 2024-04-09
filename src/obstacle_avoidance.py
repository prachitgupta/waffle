#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Obstacle:

    def __init__(self) -> None:
        
        rospy.Subscriber("/camera/depth/image_raw", Image, self.image_cb)
        self.flag_pub=rospy.Publisher("/flag",Bool,queue_size=10)
        self.flag=Bool()
        self.flag.data=False
        self.vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.vel_msg=Twist()
        self.bridge=CvBridge()
        self.count=0

    def image_cb(self,msg):

        try:
            self.cv_image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        (height,width)=self.cv_image.shape
        self.center_section=self.cv_image[:height-115,width//4:width*3//4]
        self.left_section=self.cv_image[:height-115,:width//4]
        self.right_section=self.cv_image[:height-115,width*3//4:]

        (rows,cols)=self.center_section.shape

        for i in range(rows):
            for j in range(cols):
                k=self.center_section[i,j]

                if k<=0.75:
                    self.count+=1
                
        if self.count>0.5*rows*cols:
            self.stop()
            self.flag.data=True
            
            # self.avoid()
            print(self.count)

        else:
            self.vel_msg.linear.x=0.2

        self.flag_pub.publish(self.flag)
        self.vel_pub.publish(self.vel_msg)
        cv.imshow("Original Image",self.cv_image)
        cv.imshow("Center section",self.center_section)
        cv.imshow("Left Section",self.left_section)
        cv.imshow("Right Section",self.right_section)
        cv.setMouseCallback("Center section",self.mouse_cb)
        cv.waitKey(3)

    def mouse_cb(self,event,x,y,flags,param):

        if event==cv.EVENT_MOUSEMOVE:
            print(x,y)

    def avoid(self):

        self.vel_msg.angular.z=-0.5
        self.vel_msg.linear.x=0.2

    def stop(self):

        self.vel_msg.angular.z=0.0
        self.vel_msg.linear.x=0.0
        self.vel_pub.publish(self.vel_msg)
        # rospy.sleep(0.5)

if __name__=="__main__":

    rospy.init_node("obstacle_avoidance")
    o=Obstacle()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv.destroyAllWindows()
