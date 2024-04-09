#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from math import pi
import cv2 

class Follow:

    def __init__(self) -> None:
        
        self.bridge=CvBridge()
        rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_cb)
        self.twist_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.twist_msg=Twist()
        self.req_height=60
        self.rate=rospy.Rate(100)
        self.Derivator=0
        self.Integrator=0
        self.Integrator_max=10
        self.Integrator_min=-10
        self.window_size=5

    def low_pass_filter(self,data,window_size):

        if len(data)<window_size:
            return data
        
        kernel=np.ones(window_size)/window_size
        return np.apply_along_axis(lambda x: np.convolve(x,kernel,mode='valid'),axis=0,arr=data)
        
    def vertical_bands(self,arr,nrows,ncols):

        """
        Return an array of shape (n, nrows, ncols) where
        n * nrows * ncols = arr.size
        """
        h, w = arr.shape
        assert h % nrows == 0, f"{h} rows is not evenly divisible by {nrows}"
        assert w % ncols == 0, f"{w} cols is not evenly divisible by {ncols}"
 
        return (arr.reshape(h//nrows, nrows, -1, ncols)
                .swapaxes(1,2)
                .reshape(-1, nrows, ncols))
    
    def PID(self,error,Kp,Ki,Kd):

        if error>pi:
            error=error-2*pi
        elif error<-pi:
            error=error+2*pi

        P=Kp*error

        D=Kd*(error-self.Derivator)
        self.Derivator=error

        self.Integrator=self.Integrator+error
        if self.Integrator>self.Integrator_max:
            self.Integrator=self.Integrator_max
        elif self.Integrator<self.Integrator_min:
            self.Integrator=self.Integrator_min
        I=self.Integrator*Ki

        pid=P+I+D
        return pid

    def depth_cb(self,img):

        column_size = 8
        mat_list = []
        depth_image = self.bridge.imgmsg_to_cv2(img)
        # depth_image = depth_image[self.req_height:,:]
        cv2.imshow('Img',depth_image)
        cv2.waitKey(1)
        depth_image = np.nan_to_num(depth_image,nan=20)
        depth_array = np.array(depth_image, dtype=np.float32) #424*240
        depth_array_smoothed=self.low_pass_filter(depth_array,self.window_size)
        matrix_slice = self.vertical_bands(depth_array_smoothed, 240-self.window_size+1, column_size)

        for i in range(len(matrix_slice)):
            mat_sum = sum(map(sum,matrix_slice[i])) / 10000   #adding all the elements of the sliced arrays
            mat_list.append(mat_sum)

        mat_array = np.array(mat_list)
        # mat_array = mat_array[~np.isnan(mat_array)]
        mat_array=np.nan_to_num(mat_array,nan=20)
        max_value = np.amax(mat_array)
        print(max_value)
        max_value_index = list(mat_array).index(max_value)
        print("index:", max_value_index)
        upper_middle_index = (len(mat_array) // 2)
        if abs(max_value_index-upper_middle_index)<=2:
            max_value_index=26
        print(mat_array[upper_middle_index])
        angle_error = (upper_middle_index) - (max_value_index)
        self.ang_vel = self.PID(angle_error,8,0.06,0.15)

        if abs(self.ang_vel)>0.2:
            self.ang_vel=self.ang_vel/abs(self.ang_vel)*0.2

    def pixel2depth(self):

        self.ang_vel=0
        V_cmd = 0
        omg_cmd = 0

        while not rospy.is_shutdown():
            
            V_cmd = 0.2
            omg_cmd = np.clip(self.ang_vel,-0.95,0.95) *0.9
            self.twist_msg.linear.x = V_cmd            #uncomment this when you use a ROS compatibale system as a slave node
            self.twist_msg.angular.z = omg_cmd         #uncomment this when you use a ROS compatibale system as a slave node
                
            self.twist_pub.publish(self.twist_msg)                   #uncomment this when you use a ROS compatibale system as a slave node
            self.rate.sleep()

if __name__=="__main__":

    rospy.init_node("better_row_following")

    f=Follow()
    f.pixel2depth()