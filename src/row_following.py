#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from math import pi

Derivator=0
Integrator=0
Integrator_max=10
Integrator_min=-10

def vertical_bands(arr, nrows, ncols):
    """
    Return an array of shape (n, nrows, ncols) where
    n * nrows * ncols = arr.size
    """
    h, w = arr.shape
    assert h % nrows == 0, f"{h} rows is not evenly divisible by {nrows}"
    assert w % ncols == 0, f"{w} cols is not evenly divisible by {ncols}"
    print(np.shape(arr.reshape(h//nrows, nrows, -1, ncols)
               .swapaxes(1,2)
               .reshape(-1, nrows, ncols)))
    return (arr.reshape(h//nrows, nrows, -1, ncols)
               .swapaxes(1,2)
               .reshape(-1, nrows, ncols))


'''
This function performs the following sequence actions:
1. Converts the depth image published by the camera into OpenCV image (depth_image)
2. Stores the image pixels into a 2D array (depth_array)
3. Divides the 2D array into vertical strips as defined by the user (matrix_slice)
4. The for loop adds all the elements of the respective sliced arrays and stores the sum in a list (mat_list) and then converted to a 1D array (mat_array)
5. We then find the maximum element of this array along with its index (max_value_index)
6. Next we find the index value of the middle element of the array (upper middle index in case of even number of array elements)
7. For our robot to run parallel to the crop rows, we find heading error of the robot (angle_error)
8. We then find the robots angular velocity using the Proportional controller (ang_vel)
'''

def PID(error,Kp,Ki,Kd):

    global Derivator,Integrator,Integrator_max,Integrator_min

    if error>pi:
        error=error-2*pi
    elif error<-pi:
        error=error+2*pi

    P=Kp*error

    D=Kd*(error-Derivator)
    Derivator=error

    Integrator=Integrator+error
    if Integrator>Integrator_max:
        Integrator=Integrator_max
    elif Integrator<Integrator_min:
        Integrator=Integrator_min
    I=Integrator*Ki

    pid=P+I+D

    return pid

#width(column) = 848, height(row) = 480
def convert_depth_image(ros_image):
    global ang_vel
    column_size = 8
    mat_list = []
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(ros_image)
    depth_image = np.nan_to_num(depth_image)
    depth_array = np.array(depth_image, dtype=np.float32) #848*480
    matrix_slice = vertical_bands(depth_array, 240, column_size)
    print(len(matrix_slice))

    for i in range(len(matrix_slice)):
        mat_sum = sum(map(sum,matrix_slice[i])) / 10000   #adding all the elements of the sliced arrays
        mat_list.append(mat_sum)

    mat_array = np.array(mat_list)
    mat_array = mat_array[~np.isnan(mat_array)]
    max_value = np.amax(mat_array)
    max_value_index = list(mat_array).index(max_value)
    upper_middle_index = (len(mat_array) / 2)
    angle_error = (upper_middle_index) - (max_value_index)
    print(angle_error)
    ang_vel = PID(angle_error,8,0.06,0.15)
    if abs(ang_vel)>0.2:
        ang_vel=ang_vel/abs(ang_vel)*0.2
    # rospy.loginfo("omg=%f, binid=%f ",ang_vel, max_value_index)

def pixel2depth():
    global ang_vel
    global V_cmd, omg_cmd
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image, convert_depth_image, queue_size=10)
    pos = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  #uncomment this when you use a ROS compatibale system as a slave node

    vel = Twist()
    ang_vel = 0
    V_cmd = 0
    omg_cmd = 0
    looprate=5
    rate=rospy.Rate(looprate)
    while not rospy.is_shutdown():
        
        V_cmd = 0.15
        omg_cmd = np.clip(ang_vel,-0.95,0.95) *0.95
        vel.linear.x = V_cmd            #uncomment this when you use a ROS compatibale system as a slave node
        vel.angular.z = omg_cmd         #uncomment this when you use a ROS compatibale system as a slave node
            
        #+ = acw
        pos.publish(vel)                   #uncomment this when you use a ROS compatibale system as a slave node
        rate.sleep()

if __name__ == '__main__':
    pixel2depth()