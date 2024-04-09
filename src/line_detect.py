#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nrcg.msg import Line
import numpy as np
from math import sin, cos, radians
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class LineDetectionNode:

    def __init__(self) -> None:
        
        self.threshold_distance=0.5
        self.max_iterations=100
        self.min_inliers=10
        self.angle_threshold=15.0
        self.min_distance_between_lines=0.2
        rospy.Subscriber("/scan",LaserScan,self.laser_cb)

        self.line_publisher=rospy.Publisher("/detected_lines",Marker,queue_size=10)
    
    def laser_cb(self,msg):

        ranges=np.array(msg.ranges)
        valid_ranges=np.isfinite(ranges)
        valid_indices=[]
        for i,j in enumerate(valid_ranges):
            if j:
                valid_indices.append(i)
        
        lines=self.ransac_line_detection(ranges,valid_indices)

    def ransac_line_detection(self,scan_ranges,valid_indices):

        best_line=Line()
        best_inliers=[]
        
        for _ in range(self.max_iterations):

            sample_indices=np.random.choice(valid_indices,size=2,replace=False)

            theta_1=sample_indices[0]
            theta_2=sample_indices[1]

            p1=scan_ranges[theta_1]
            p2=scan_ranges[theta_2]

            x1=p1*cos(radians(theta_1))
            y1=p1*sin(radians(theta_1))

            x2=p2*cos(radians(theta_1))
            y2=p2*sin(radians(theta_1))

            slope=(y2-y1)/(x2-x1)
            intercept=y1-slope*x1

            distances=np.abs(scan_ranges-(slope*np.arange(len(scan_ranges))+intercept))

            inliers=np.where(distances<self.threshold_distance)[0]

            if len(inliers)>len(best_inliers):
                best_line=(inliers[0],inliers[-1])
                best_inliers=inliers
            
        if len(best_inliers)<self.min_inliers:
            return []

        self.visualize_lines(scan_ranges[best_line[0]],best_line[0],scan_ranges[best_line[1]],best_line[1])
        print(best_line)
        return[best_line] 
    
    def visualize_lines(self,x1,y1,x2,y2):

        line_msg = Marker()
        line_msg.header.stamp = rospy.Time.now()
        line_msg.header.frame_id = "base_scan"  # Update with your laser frame
        line_msg.type = Marker.LINE_STRIP
        line_msg.action = Marker.ADD
        line_msg.scale.x = 0.05  # Line width
        line_msg.color.a = 1.0
        line_msg.color.r = 1.0
        line_msg.points.append(Point(x=x1, y=y1))
        line_msg.points.append(Point(x=x2, y=y2))

        self.line_publisher.publish(line_msg)
        
if __name__=="__main__":

    rospy.init_node("laser_detect_node")
    try:
        l=LineDetectionNode()
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass
