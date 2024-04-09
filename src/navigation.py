#!/usr/bin/env python
"""
navigation algorithm using CV and ML combined together
@version: 2.0 
@author: Aghi Diego
"""

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from waffle.msg import Tupla

import rospy
import numpy as np
import cv2, os
import time
from PIL import Image as IMM
from keras.models import load_model 


import threading
import tf.transformations as trans

#working height and weight
w_depth=320
h_depth=240




class INIT():

	def __init__(self):
		rospy.init_node('ROS_API')
		global ranges, pose, angular_velocity, linear_acceleration, image, imgRGB,ang_vel_command 
		image = np.zeros((480, 640), np.uint8)
		ang_vel_command=0
	
		imgRGB = np.zeros((224, 224, 3), np.uint8)
		
class OUTPUT():

	def __init__(self, cmd_vel='/cmd_vel'):
		self.pub_vel = rospy.Publisher(cmd_vel, Twist, queue_size=10)
		self.pub_tupla = rospy.Publisher('control_value', Tupla, queue_size=10)
		# rate = rospy.Rate(2)
		self.vel = Twist()
		self.tupla = Tupla()

	def Move(self, linear_x, angular_theta):
		'''
		Publish the components of velocity
		:param linear_x: 
		:param angular_theta:
		:return: none
		'''
		self.vel.linear.x=linear_x
		self.vel.angular.z=angular_theta
		self.pub_vel.publish(self.vel)

	def Communication(self, flag, controller_data):
		'''
		Publish the results for the controllers
		:param flag_choice 0 DEPTH 1 ML: 
		:param distance or prediction:
		:return: none
		'''
		self.tupla.flag=flag
		self.tupla.control=controller_data
		self.pub_tupla.publish(self.tupla)
		
		



class INPUT(threading.Thread):

	def __init__(self, scan='/scan', odom='/odom', imu='/imu', camera='/camera/depth/image_raw', cameraRGB='/camera/rgb/image_raw'):
		threading.Thread.__init__(self)
		self.scan = scan
		self.odom = odom
		self.imu = imu
		self.camera = camera
		self.cameraRGB = cameraRGB

	def run(self):
		self.sub_lid = rospy.Subscriber(self.scan, LaserScan, self.Lidar)
		self.sub_odom = rospy.Subscriber(self.odom, Odometry, self.Odometry)
		self.sub_odom = rospy.Subscriber(self.imu, Imu, self.Imu)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(self.camera, Image, self.Camera)
		self.imageRGB_sub = rospy.Subscriber(self.cameraRGB, Image, self.CameraColor)
		rospy.spin()

	def Lidar(self, msg):
		'''
		Control the LiDAR inputs
		:param msg:
		:return: ranges of the 360 values as numpy array
		'''
		global ranges
		ranges = msg.ranges

	def Odometry(self, msg):
		'''
		Control the odometry from the robot
		:param msg:
		:return: pose [coordinates, euler angles] as numpy array
		'''
		global pose
		position = msg.pose.pose.position
		orientation = msg.pose.pose.orientation

		coordinates = [position.x, position.y, position.z]
		euler = list(trans.euler_from_quaternion(
			(orientation.x, orientation.y, orientation.z, orientation.w)
			))

		pose = [coordinates, euler] # np.asarray([coordinates, euler])
		# yaw = euler[2] * 180 / math.pi

	def Imu(self, msg):
		'''
		Control the Inertia Measurement Unit from the robot
		:param msg:
		:return: angular velocity and linear acceleration as numpy arrays
		'''
		global angular_velocity, linear_acceleration
		angular_velocity = np.asarray([msg.angular_velocity.x,
									   msg.angular_velocity.y,
									   msg.angular_velocity.z])
		linear_acceleration = np.asarray([msg.linear_acceleration.x,
										  msg.linear_acceleration.y,
										  msg.linear_acceleration.z])

	def Camera(self, data):
		global image
		#image = self.bridge.imgmsg_to_cv2(data, "16UC1")
		image = cv2.resize(self.bridge.imgmsg_to_cv2(data, "16UC1"), (w_depth,h_depth)) #resizing image to reduce complexity and computational load

	def CameraColor(self, data):
		global imgRGB

		imgRGB = cv2.resize(self.bridge.imgmsg_to_cv2(data, "bgr8"), (224,224)) #resizing img to make it fit with the ML model 
		


#to find the window where all the points are beyond a certain depth. Basically just thresholding the frame

def findWindow(frame):
	threshold=0.41 #empirical threshold
	r,col=frame.shape


	frame=np.where(frame>threshold, 1.0, 0)
	return frame


'''
#function to set the controller
def controllerDepth(delta):
		print("delta",delta)
		#parabolic control 
		if delta > 0:								#left side, turn right
			ang_vel_command = -float(max_Ang_Vel*delta*delta)/((w_depth/2)*(w_depth/2))

		else:										#right side, turn left
			ang_vel_command = float(max_Ang_Vel*delta*delta)/((w_depth/2)*(w_depth/2))


		print("ang_vel_command",ang_vel_command)
		return ((max_Ang_Vel-abs(ang_vel_command)),ang_vel_command)



'''
def loss_IoU(y_true, y_pred):
    y_true = tf.cast(y_true, tf.float32)
    y_pred = tf.cast(y_pred > 0.5, tf.float32)  # Convert predictions to binary mask
    intersection = tf.reduce_sum(y_true * y_pred, axis=[1, 2])
    union = tf.reduce_sum(y_true, axis=[1, 2]) + tf.reduce_sum(y_pred, axis=[1, 2]) - intersection
    iou = tf.reduce_mean((intersection + 1e-7) / (union + 1e-7))  # Add a small epsilon to avoid division by zero
    return 1 - iou

#load the desired ML model
def deepVineyardModel(pathModel):
	model = load_model(pathModel,custom_objects = {'loss_IoU': loss_IoU})
	return model


if __name__ == '__main__':
    
	timer= []
	real_path = os.path.dirname(os.path.realpath(__file__))

	areaThreshold=3000 #threshold of the areas to be considered. Area of point beyond a certain threshold/depth 
	bridge=CvBridge()
	init = INIT()


	#model path definition
	model=deepVineyardModel(os.path.join(real_path,'MobileNetv3_segmentation_new1.h5'))
	#pre defined classes/labels
	classes = ['left', 'center', 'right']


	#IN = INPUT(camera="/camera/depth/image_rect_raw")
	IN = INPUT(camera="/camera/depth/image_raw" , cameraRGB="/camera/rgb/image_raw")
	IN.start()


	OUT = OUTPUT()
    
    
	while not rospy.is_shutdown():   
		image = np.array(image, dtype=np.float32)   #img conversion
		cv2.normalize(image, image, 0, 1, cv2.NORM_MINMAX) #normalization 0 to 1
		comp_algo_view=image.copy() #just to see the img

		image=findWindow(image) #function to find the rectangle/window - the navigation goal
		img_cast=image.astype("uint8") #casting of the img to do the needed following ops
		
		__,contours,hierarchy = cv2.findContours(img_cast, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #finding the contours
		area=np.zeros(len(contours)) #init the vectors

		for ind in range(len(contours)):
			
			
			cnt=contours[ind]
			area[ind] = cv2.contourArea(cnt)

		index_max_rect= np.argmax(area)  #find biggest area and get rid of the noise
		cnt2=contours[index_max_rect]
		if (area[index_max_rect]>areaThreshold):
			xx,yy,ww,hh = cv2.boundingRect(cnt2)
			cv2.rectangle(comp_algo_view,(xx,yy),(xx+ww,yy+hh),(1,255,255),2)
			OUT.Communication(0,(xx+(ww/2)-((w_depth/2)-1)))
		else:
			just_view_=imgRGB.copy()	
			imgRGB=(imgRGB/255.)
			y_pred = model.predict(imgRGB[None,...])
			ML_predict = np.argmax(y_pred, axis=-1)[0]
			OUT.Communication(1,ML_predict)

			cv2.imshow("image my",just_view_)
		cv2.imshow("depth",comp_algo_view)
            # cv2.rectangle(comp_algo_view,(xx,yy),(xx+ww,yy+hh),(1,255,255),2)		
				
				# OUT.Communication(0,(xx+(ww/2)-((w_depth/2)-1))) # sending command to the control module

		    # else:
                
			# 	just_view_=imgRGB.copy() #just to see the img at the end
		   	# 	#scale the img
			# 	imgRGB=(imgRGB/255.)
		   	# 	y_pred = model.predict(imgRGB[None,...]) #adding one dimension
		   	# 	ML_predict=np.argmax(y_pred, axis=-1)[0] #reading model prediction
		   	# 	OUT.Communication(1,ML_predict)  # sending command to the control module

		   	# 	cv2.imshow('ROS API color', just_view_)
		   

		    # cv2.imshow('depth View', comp_algo_view)
		k = cv2.waitKey(1) & 0xFF
		if k == 2:
			OUT.move(0,0)
			break

			
		  
		# except Exception as e:
            
		# 	print(e)
		
	cv2.destroyAllWindows()
	rospy.signal_shutdown('Closing the program...')
