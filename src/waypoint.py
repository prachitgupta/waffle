#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi,sqrt,atan2
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

#straight and dubins
# WAYPOINTS=[[6.0,0.0],[6.6816387600233345,-0.2683111311261791],[6.997494986604054,-0.9292627983322971],[6.778073196887921,-1.628173622722739],[6.0,-2.0]]

#circle
# WAYPOINTS=[[3.0, 0.0], [2.9630650217854133, 0.46930339512069263], [2.8531695488854605, 0.9270509831248421], [2.673019572565104, 1.3619714992186402], [2.4270509831248424, 1.7633557568774194], [2.121320343559643, 2.1213203435596424], [1.7633557568774194, 2.4270509831248424], [1.3619714992186405, 2.6730195725651034], [0.9270509831248424, 2.8531695488854605], [0.46930339512069275, 2.9630650217854133], [1.8369701987210297e-16, 3.0], [-0.4693033951206924, 2.9630650217854133], [-0.927050983124842, 2.853169548885461], [-1.36197149921864, 2.673019572565104], [-1.7633557568774192, 2.4270509831248424], [-2.1213203435596424, 2.121320343559643], [-2.427050983124842, 1.7633557568774196], [-2.6730195725651034, 1.3619714992186407], [-2.8531695488854605, 0.9270509831248426], [-2.963065021785413, 0.46930339512069297], [-3.0, 3.6739403974420594e-16], [-2.9630650217854133, -0.4693033951206922], [-2.853169548885461, -0.9270509831248419], [-2.673019572565104, -1.36197149921864], [-2.427050983124843, -1.7633557568774192], [-2.121320343559643, -2.1213203435596424], [-1.7633557568774196, -2.427050983124842], [-1.3619714992186407, -2.6730195725651034], [-0.9270509831248427, -2.8531695488854605], [-0.4693033951206931, -2.963065021785413], [-5.51091059616309e-16, -3.0], [0.469303395120692, -2.9630650217854133], [0.9270509831248417, -2.853169548885461], [1.3619714992186398, -2.673019572565104], [1.7633557568774187, -2.427050983124843], [2.121320343559642, -2.121320343559643], [2.427050983124842, -1.76335575687742], [2.6730195725651034, -1.361971499218641], [2.8531695488854605, -0.9270509831248428], [2.963065021785413, -0.46930339512069336], [3.0, -7.347880794884119e-16]]

#Lemniscate of Bernoulli(infinity curve)
# WAYPOINTS=[[4.242640687119286, 0.0], [4.090309736045309, 0.6398654154070958], [3.6832700768782325, 1.1381930486280927], [3.1342321703346485, 1.422911629309991], [2.5510145621368183, 1.4994487380073627], [2.0000000000000004, 1.4142135623730951], [1.5072522328558045, 1.2193926711899312], [1.073708948733704, 0.9566816784011638], [0.6883918214503026, 0.6546995275546001], [0.3359583562068407, 0.3318221513510047], [1.29893408435324e-16, 1.29893408435324e-16], [-0.3359583562068405, -0.33182215135100446], [-0.6883918214503024, -0.6546995275546], [-1.0737089487337035, -0.9566816784011637], [-1.5072522328558038, -1.2193926711899308], [-2.0, -1.4142135623730951], [-2.5510145621368174, -1.4994487380073624], [-3.1342321703346476, -1.422911629309991], [-3.6832700768782325, -1.1381930486280931], [-4.090309736045308, -0.6398654154070961], [-4.242640687119286, -5.19573633741296e-16], [-4.090309736045309, 0.6398654154070952], [-3.6832700768782334, 1.1381930486280927], [-3.1342321703346485, 1.422911629309991], [-2.5510145621368188, 1.4994487380073624], [-2.0000000000000004, 1.4142135623730951], [-1.5072522328558047, 1.2193926711899312], [-1.0737089487337041, 0.956681678401164], [-0.6883918214503029, 0.6546995275546004], [-0.335958356206841, 0.33182215135100496], [-3.89680225305972e-16, 3.89680225305972e-16], [0.33595835620684017, -0.3318221513510042], [0.688391821450302, -0.6546995275545997], [1.0737089487337033, -0.9566816784011637], [1.5072522328558033, -1.2193926711899306], [1.9999999999999993, -1.414213562373095], [2.5510145621368174, -1.499448738007363], [3.1342321703346476, -1.4229116293099915], [3.6832700768782316, -1.1381930486280933], [4.090309736045307, -0.6398654154070965], [4.242640687119286, -1.039147267482592e-15]]

#square
# WAYPOINTS=[[3,0],[3,3],[0,3],[0,0]]

#NRCG world
# WAYPOINTS=[[3.2,-6],[0,-6],[0,6],[-3.2,6],[-3.2,-6]]

#test
WAYPOINTS=[[6,0],[2,0]]

class PID:

    def __init__(self,P=0.0,I=0.0,D=0.0,Derivator=0,Integrator=0,Integrator_max=10,Integrator_min=-10):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):

        self.error=self.set_point-current_value
        if self.error>pi:
            self.error=self.error-2*pi
        elif self.error<-pi:
            self.error=self.error+2*pi

        self.P_value=self.Kp*self.error

        self.D_value=self.Kd*(self.error-self.Derivator)
        self.Derivator=self.error

        self.Integrator=self.Integrator+self.error
        if self.Integrator>self.Integrator_max:
            self.Integrator=self.Integrator_max
        elif self.Integrator<self.Integrator_min:
            self.Integrator=self.Integrator_min
        self.I_value=self.Integrator*self.Ki

        PID=self.P_value+self.I_value+self.D_value
        
        return PID
    
    def setPoint(self,set_point):
        
        self.set_point=set_point
        self.Derivator=0
        self.Integrator=0

    def setPID(self,set_P=0.0,set_I=0.0,set_D=0.0):

        self.Kp=set_P
        self.Ki=set_I
        self.Kd=set_D

class Move():

    def __init__(self):

        self.threshold=0.8

        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.pid_theta=PID(0,0,0)

        #for turtlebot3
        self.odom_sub=rospy.Subscriber("/odom",Odometry,self.odom_cb)
        self.twist_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        
        #for warthog
        # self.odom_sub=rospy.Subscriber("/odometry/filtered",Odometry,self.odom_cb)
        # self.twist_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        rospy.Subscriber("/scan",LaserScan,self.scan_cb)

        self.vel=Twist()
        self.rate=rospy.Rate(10)

        for point in WAYPOINTS:
            self.move_to_point(point[0],point[1])
        self.stop()

    def move_to_point(self,x,y):

        diff_x=x-self.x
        diff_y=y-self.y
        direction_vector=np.array([diff_x,diff_y])
        direction_vector=direction_vector/sqrt(diff_x**2+diff_y**2)
        theta=atan2(diff_y,diff_x)

        if theta>-pi and theta<-3.13:
            theta=theta+2*pi
            print("theta corrected")

        self.pid_theta.setPID(8,0.06,0.15)
        self.pid_theta.setPoint(theta)

        while not rospy.is_shutdown():

            angular=self.pid_theta.update(self.theta)
            if abs(angular)>0.2:
                angular=angular/abs(angular)*0.2
            if abs(angular)<0.01:
                break
            
            print(self.theta)
            self.vel.linear.x=0
            self.vel.angular.z=angular
            self.twist_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()

        while not rospy.is_shutdown():

            if self.flag==True:
                self.stop()
                break
            
            diff_x=x-self.x
            diff_y=y-self.y
            yaw_actual=self.theta
            vector=np.array([diff_x,diff_y])
            linear=np.dot(vector,direction_vector)

            if abs(linear)>0.2:
                linear=linear/abs(linear)*0.2

            if abs(linear)<0.01 and abs(angular)<0.01:
                break

            if yaw_actual>-pi and yaw_actual<-2.95:
                yaw_actual=yaw_actual+2*pi
                print("yaw corrected")

            if abs(yaw_actual-theta)>0.005:
                angular=-1*np.sign(yaw_actual-theta)*0.1
            else:
                angular=0.0

            print(yaw_actual,theta)
            self.vel.linear.x=linear
            self.vel.angular.z=angular
            self.twist_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()

    def stop(self):

        self.vel.linear.x=0.0
        self.vel.angular.z=0.0
        self.twist_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_cb(self,msg):

        quat=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        (roll,pitch,yaw)=tf.transformations.euler_from_quaternion(quat)
        self.theta=yaw
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y

    def scan_cb(self,msg):

        if msg.ranges[0]<self.threshold:
            self.flag=True

        else:
            self.flag=False
           
           
if __name__=="__main__":

    rospy.init_node("waypoint")

try:
    Move()
except rospy.ROSInterruptException:
    rospy.loginfo("Mission aborted")