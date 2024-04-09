#!/usr/bin/env python3
"""
navigation using only machine learning model

@author: Aghi Diego
"""

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as IMM
from keras.models import load_model 
from waffle.msg import Tupla
import rospy
import numpy as np
import cv2, os
import threading
import tf.transformations as trans

class INIT():

	def __init__(self):
		rospy.init_node('ROS_API')
		global ranges, pose, angular_velocity, linear_acceleration, image, imgRGB
		image = np.zeros((480, 640), np.uint8)
		imgRGB = np.zeros((224, 224, 3), np.uint8)
		#224 224 for MobileNet , 299 299 for Xception
class OUTPUT():

	def __init__(self, cmd_vel='/cmd_vel'):
		self.pub_vel = rospy.Publisher(cmd_vel, Twist, queue_size=10) ##same for waffle
		self.pub_Tupla = rospy.Publisher('control_value', Tupla, queue_size=10)
		# rate = rospy.Rate(2)
		self.vel = Twist()
		self.Tupla = Tupla()

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
		self.Tupla.flag=flag
		self.Tupla.control=controller_data
		self.pub_Tupla.publish(self.Tupla)
		##ssend to controller


class INPUT(threading.Thread):

	def __init__(self, scan='/scan', odom='/odom', imu='/imu', cameraRGB='/camera/rgb/image_raw'):
		threading.Thread.__init__(self)
		self.scan = scan
		self.odom = odom
		self.imu = imu
		self.cameraRGB = cameraRGB

	def run(self):  ##all these run in backhand getting data and sending to topic
		self.sub_lid = rospy.Subscriber(self.scan, LaserScan, self.Lidar)
		self.sub_odom = rospy.Subscriber(self.odom, Odometry, self.Odometry)
		self.sub_odom = rospy.Subscriber(self.imu, Imu, self.Imu)
		self.bridge = CvBridge()
		#self.image_sub = rospy.Subscriber(self.camera, Image, self.Camera)
		self.imageRGB_sub = rospy.Subscriber(self.cameraRGB, Image, self.CameraColor)
		print("i am trying")
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


	def CameraColor(self, data):
		global imgRGB
		#resize img to network input shape
		imgRGB = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Resizing and normalization
		imgRGB = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2RGB)
		imgRGB = cv2.resize(imgRGB.astype('uint8'), (224,224), interpolation = cv2.INTER_AREA)
		imgRGB = (imgRGB/255.)
	

#load the desired ML model

# def setInput(interpreter, data):
# 	"""Copies data to input tensor."""
# 	input_tensor(interpreter)[:, :] = data
	
# def output_tensor(interpreter):
# 	"""Returns dequantized output tensor."""
# 	output_details = interpreter.get_output_details()[0]
# 	output_data = interpreter.tensor(output_details['index'])()
# 	return output_data
	
# def input_tensor(interpreter):
# 	"""Returns input tensor view as numpy array of shape (height, width, 3)."""
# 	tensor_index = interpreter.get_input_details()[0]['index']
# 	return interpreter.tensor(tensor_index)()[0]


def loss_IoU(y_true, y_pred):
    y_true = tf.cast(y_true, tf.float32)
    y_pred = tf.cast(y_pred > 0.5, tf.float32)  # Convert predictions to binary mask
    intersection = tf.reduce_sum(y_true * y_pred, axis=[1, 2])
    union = tf.reduce_sum(y_true, axis=[1, 2]) + tf.reduce_sum(y_pred, axis=[1, 2]) - intersection
    iou = tf.reduce_mean((intersection + 1e-7) / (union + 1e-7))  # Add a small epsilon to avoid division by zero
    return 1 - iou



#load the desired ML model
def deepVineyardModel(pathModel):
	model = load_model(pathModel,custom_objects = {'loss_IoU': loss_IoU,'class_IoU': class_IoU})
	return model

if __name__ == '__main__':
	
	#model path definition
	real_path = os.path.dirname(os.path.realpath(__file__)) ##path of directory with python script
	model=deepVineyardModel(os.path.join(real_path,'mobileNetv3_segmentation_new1.h5'))
	#pre defined classes/labels
	classes = ['left', 'center', 'right']


	bridge=CvBridge()
	init = INIT()

	IN2 = INPUT(cameraRGB="/camera/rgb/image_raw")
	#IN2.run()
	IN2.start()

	OUT = OUTPUT()
    
	while not rospy.is_shutdown():
		try:
			#model input
			y_pred = model.predict(imgRGB[None,...]) #adding one dimension
			ML_predict = np.argmax(y_pred, axis=-1)[0] 
			cv2.imshow('ROS API color', imgRGB)
			OUT.Communication(1,ML_predict)
			k = cv2.waitKey(1) & 0xFF
   
		except Exception as e:
			print(e)
        ##send output to controller
            
    
 
	# while not rospy.is_shutdown():

	# 		#model input
	# 	   setInput(interpreter, imgRGB[None,...])
	# 	   print("i am trying")
	# 		# invoke interpreter
	# 	   interpreter.invoke()
	# 	   y_pred = output_tensor(interpreter)[0]
	# 	   ML_predict = np.argmax(y_pred > 0.85, axis=-1)[0]
	# 	   cv2.imshow('ROS API color', imgRGB)
	# 	   OUT.Communication(1,int(ML_predict))   #SENDING COMMANDS to controller
		  

		
	# 	   k = cv2.waitKey(1) & 0xFF
     
	#out.release()
	cv2.destroyAllWindows()
	rospy.signal_shutdown('Closing the program...')