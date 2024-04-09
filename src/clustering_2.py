#!/usr/bin/env python3

import rospy
import cv2 
import numpy as np
from sklearn.cluster import KMeans
import random
import math
import imutils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Clustering:

    def __init__(self):
        
        self.rgb_image = np.zeros((1080,1920,3))
        self.depth_image = np.zeros((1080,1920))
        self.callback_trigger = False
        self.bridge = CvBridge()

    def remove_ground_2(self):
        
        h,w= np.shape(self.depth_image)
        divided_array = np.array_split(self.depth_image[int(h/2):h,:],86,axis=1)
        i = 0
        mask = np.zeros((int(h/2),w))
        angular_width = h/56

        for x in divided_array:
            horizontal_split_avg = np.nanmean(x,axis=1)
            indices = np.arange(0,h/2)
            valid_indices = np.where((np.cos(((90 -(indices/angular_width))*math.pi/180))*horizontal_split_avg) < 0.1)
            mask[valid_indices,int(i*4.9):int(i*4.9) + 5] = 1
            i += 1
        
        cv2.imshow("mask_2",self.depth_image)
        cv2.waitKey(1)
        return mask

    def cluster_detection(self):

        # Set depth threshold for feature segmentation
        depth_threshold = 0.1 # please change this value with your own preference
        h,w = np.shape(self.depth_image)
        # Generate binary mask based on depth threshold
        binary_mask = np.where(self.depth_image > depth_threshold, 255, 0).astype(np.uint8)
        mask_depth = self.remove_ground_2()
        self.depth_image[int(h/2):,:] = self.depth_image[int(h/2):,:]*mask_depth

        self.rgb_image[int(h/2):,:,0] = self.rgb_image[int(h/2):,:,0]*mask_depth
        self.rgb_image[int(h/2):,:,1] = self.rgb_image[int(h/2):,:,1]*mask_depth
        self.rgb_image[int(h/2):,:,2] = self.rgb_image[int(h/2):,:,2]*mask_depth

        # Reshape the image to a 2D array of pixels
        pixels = self.depth_image.reshape((-1, 1))
        # Perform K-means clustering on the pixel values
        num_clusters = 1
        kmeans = KMeans(n_init='auto')
        kmeans.fit(pixels)

        # Get the labels and cluster centers
        labels = kmeans.labels_
        centers = kmeans.cluster_centers_
        # Reshape the labels to match the image shape
        labels = labels.reshape(self.depth_image.shape)

        # Extract cluster features from the RGB image
        cluster_features = []
        mask = np.zeros((h,w),dtype=np.uint8)
        
        for x in range(len(centers)):
            r = random.randint(0, 255)
            g = random.randint(0, 255)
            b = random.randint(0, 255)
            mask = np.where(labels == x, 255, 0).astype(np.uint8)
            cluster_image = cv2.bitwise_and(self.depth_image, self.depth_image, mask=mask)
        
            if len(cluster_image[np.nonzero(cluster_image)]) > 2000:
                if np.min(cluster_image[np.nonzero(cluster_image)]) < 4 and np.average(cluster_image) > 0.1:
                    cluster_features.append(cluster_image) 
                    self.rgb_image[np.where(mask==255)] = [255,0,0]
                    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    cnts = imutils.grab_contours(cnts)
                    print((np.min(np.nonzero(cluster_image)))*10/255)
                    
                    for c in cnts:
                        if cv2.contourArea(c) > 1000:
                            M = cv2.moments(c)

                            if M['m00'] != 0:
                                cX = int(M["m10"] / M["m00"])
                                cY = int(M["m01"] / M["m00"])
                                print(self.depth_image[cY,cX])
                                cv2.circle(self.rgb_image, (cX, cY), 7, (0, 0, 0), -1)
                                cv2.putText(self.rgb_image, "("+ str(cX) + "," + str(cY) + ")", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # mask_depth *=255
        cv2.imshow("RealSense",self.rgb_image)
        cv2.waitKey(1)
    
    def depth_callback(self,data):
        
        self.depth_image = self.bridge.imgmsg_to_cv2(data)
        self.depth_image = np.nan_to_num(self.depth_image)
        # print(self.depth_image)
        self.cluster_detection()
        
    def color_callback(self,data):
        
        self.rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

    def start(self):
        
        rospy.init_node("Clustering")
        rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
        rospy.Subscriber("/camera/rgb/image_raw",Image,self.color_callback)
        rospy.spin()

if __name__=="__main__":

    try:
        cluster_detect = Clustering()
        cluster_detect.start()
    except:
        exit()