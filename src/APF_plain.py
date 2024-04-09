#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import numpy as np
from math import *
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from PID_control import PID_control

class APF():
    """
    Class for planning based on artificial potential fields
    """
    def __init__(self,start:np.array, end:np.array, obstacles: np.array, attractive_gain=5,
                 Repulsive_gain=100, robot_radius = np.array([1,2,1,2]), step_size = 0.0001, max_iters=1000 ):
        
        self.start = start
        self.goal = end
        self.current = self.start
        self.q_star = robot_radius
        self.obstacles = obstacles
        self.eta = Repulsive_gain
        self.zeta = attractive_gain
        self.step = step_size
        self.max_iter = max_iters
        self.epsilon = 0.1
        self.PID = PID_control("OM_NAMAH_SHIVAY")
        self.path = [list(self.start)]
        self.gradient_descent()
        self.plotter()
        self.GO()
    
    def attractive_force(self, goal=None, current=None):
        # if current is None:
        #     current = self.start
        # if goal is None:
        #     goal = self.goal
        return self.zeta*(goal-current)
    
    def repulsive_force(self, obs:np.array, current=None):
        """ Repulsive force"""
        #Finding closest obstacle first
        # if current is None:
        #     current = self.start
        dmin = inf
        min_indx = -1
        for idx, ob in enumerate(obs):
            if np.linalg.norm(ob-current) < dmin:
                dmin = np.linalg.norm(ob-current)
                min_indx = idx
        if np.linalg.norm(obs[min_indx]-current) < self.q_star[min_indx]:
            dq = np.linalg.norm(obs[min_indx]-current)
            q = obs[min_indx] - current
            rep_force = q*self.eta*(1/self.q_star[min_indx] - 1/dq)/dq**2
        else:
            rep_force = np.array([0,0])
        return rep_force

    def gradient_descent(self):
        
        iters = 0
        while (iters < self.max_iter and np.linalg.norm(self.current - self.goal) > self.epsilon):

            force = self.attractive_force(self.goal,self.current) + self.repulsive_force(self.obstacles,self.current)
            self.current += force*self.step
            # print(self.current)
            self.path.append(list(self.current))
        self.path = np.array(self.path)
    
    def GO(self):
        i=0
        while self.path.shape[0] >=1:
            while np.linalg.norm([self.path[0][0] - self.PID.bot_location.x, self.path[0][1] - self.PID.bot_location.y])>self.epsilon:
                self.PID.pub_cmd_vel.publish(self.PID.Velocity_tracking_law(self.path[0]))
                print(i)
                i+=1
            self.path = np.delete(self.path,0,axis=0)
        self.PID.pub_cmd_vel.publish(Twist())

    def plotter(self):       
        plt.scatter(obstacles[:,0],obstacles[:,1],marker="o",color="r")
        plt.scatter(-2,-4,marker="x")
        plt.scatter(2,5,marker="p")
        plt.plot(self.path[:,0],self.path[:,1])
        plt.show()
        
if __name__ == '__main__':
    rospy.init_node("APF_Planner")
    start = np.array([-2.,-4])
    goal = np.array([2.,5.])
    obstacles = np.array(np.array([[0.,2.],[1.5,0.],[0.,-2.],[-1.5,0.]]))
    apf = APF(start,goal,obstacles)
    r=rospy.Rate(10)
    while not rospy.is_shutdown:
        r.sleep()

