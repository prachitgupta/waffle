#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Square():

    def __init__(self):

        self.twist_pub=rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist_msg=Twist()
        self.end_msg=Twist()
        #self.twist_msg.linear.x=0.5
        self.desired_x=5.0
        self.desired_y=5.0
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

    def odom_cb(self,msg):

        theta_r=math.atan2((self.desired_y-msg.pose.pose.position.y),(self.desired_x-msg.pose.pose.position.x))
        self.theta_d=math.degrees(theta_r)
        (roll,pitch,yaw)=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        yaw=math.degrees(yaw)
        print(self.theta_d,yaw)
        if abs(self.theta_d-yaw)>=1.0:
            self.twist_msg.angular.z=0.5
            self.twist_pub.publish(self.twist_msg)
        
        else:
            self.stop()
    
    def stop(self):

        self.twist_msg.linear.x=0.0    
        self.twist_msg.angular.z=0.0
        self.twist_pub.publish(self.twist_msg)

if __name__=="__main__":

    rospy.init_node("square")
    rate=rospy.Rate(10)
    s=Square()

    while not rospy.is_shutdown():
        
        rate.sleep()