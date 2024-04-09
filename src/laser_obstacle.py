#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class Obstacle:

    def __init__(self) -> None:
        
        rospy.Subscriber("/scan",LaserScan,self.scan_cb)
        self.flag_pub=rospy.Publisher("/flag",Bool,queue_size=10)
        self.flag=False
        self.threshold=0.8

    def scan_cb(self,data):

        if data.ranges[0]<self.threshold:
            self.flag=True

        else:
            self.flag=False
        
        self.flag_pub.publish(self.flag)

if __name__=="__main__":

    rospy.init_node("laser_obstacle")
    o=Obstacle()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission aborted")