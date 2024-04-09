#!/usr/bin/env python3
import rospy
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
import threading

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  ##
from itertools import count

import time

index = count()

np.set_printoptions(suppress=True)


pos = gmsg.Point()
pos.x = 0.0 # guarantees no instant end of path planning
pos.y = 0.0

kp_omega, kp_lin = 6, 1
ki = 0.0
kd = 0.0
error_previous = 0.0
time_current = 0
time_previous = 0
Wmax = np.pi/2
Vmax = 1   #2
theta =  0.001


#Linear Controller Gain
c1, c2, c3 = 2, 0.01, 1.1

# square
#goal_x = [4.0, 4.0, 0.0, 0.0]
#goal_y = [0.0, 4.0, 4.0,0.0]

# circle
#goal_x = [3.0983866769659336, 2.9305075405299146, 2.445062440567896, 1.6946568861175848, 0.7606089627455443, -0.2558627438040414, -1.244607751942933, -2.098480198079744, -2.7249497534794376, -3.056128721137748, -3.0561287211377484, -2.724949753479438, -2.098480198079746, -1.2446077519429348, -0.25586274380404284, 0.760608962745543, 1.6946568861175832, 2.445062440567895, 2.9305075405299146, 3.0983866769659336]
#goal_y = [0.0, 1.0060445094017016, 1.9030684858207716, 2.5938654626511095, 3.003576868633654, 3.0878041156027156, 2.837419874428802, 2.279557162754906, 1.4746690615227378, 0.5099776856264929, -0.5099776856264921, -1.4746690615227374, -2.2795571627549047, -2.8374198744288015, -3.0878041156027156, -3.0035768686336546, -2.59386546265111, -1.9030684858207723, -1.0060445094017025, -7.58885865293427e-16]

# straight and dubins
#goal_x = [6.0, 6.099833416646828, 6.198669330795061, 6.295520206661339, 6.389418342308651, 6.479425538604203, 6.564642473395035, 6.644217687237691, 6.717356090899523, 6.783326909627483, 6.841470984807897, 6.891207360061435, 6.932039085967226, 6.963558185417193, 6.98544972998846, 6.997494986604054, 6.999573603041505, 6.991664810452469, 6.973847630878195, 6.946300087687415, 6.909297426825681, 6.863209366648873, 6.80849640381959, 6.74570521217672, 6.675463180551151, 6.5984721441039556, 6.515501371821463, 6.427379880233829, 6.334988150155904, 6.2392493292139815, 6.141120008059866, 6.0]

#goal_y = [0.0, -0.0049958347219741794, -0.019933422158758374, -0.04466351087439402, -0.0789390059971149, -0.12241743810962724, -0.17466438509032167, -0.2351578127155115, -0.3032932906528345, -0.3783900317293355, -0.45969769413186023, -0.5464038785744225, -0.6376422455233264, -0.7325011713754126, -0.8300328570997592, -0.9292627983322973, -1.029199522301289, -1.1288444942955251, -1.2272020946930875, -1.323289566863504, -1.4161468365471428, -1.5048461045998578, -1.5885011172553463, -1.6662760212798249, -1.737393715541246, -1.8011436155469343, -1.8568887533689478, -1.9040721420170617, -1.9422223406686585, -1.9709581651495909, -1.9899924966004456, -2.0]

#goal_x = [6.0, 6.6816387600233345, 6.997494986604054, 6.778073196887921, 6.0]
#goal_y = [0.0, -0.2683111311261791, -0.9292627983322971, -1.628173622722739, -2.0]

#goal_x = [6.0]
#goal_y = [0.0]

goal_x = [5.0,5.09983342,5.19866933,5.29552021,5.38941834,5.47942554,5.56464247,5.64421769,5.71735609,5.78332691,5.84147098,5.89120736,5.93203909,5.96355819,5.98544973,5.99749499,5.9995736,5.99166481,5.97384763,5.94630009,5.90929743,5.86320937,5.8084964,5.74570521,5.67546318,5.59847214,5.51550137,5.42737988,5.33498815,5.23924933,5.14112001,5.0]
goal_y = [4.0,3.99500417,3.98006658,3.95533649,3.92106099,3.87758256,3.82533561,3.76484219,3.69670671,3.62160997,3.54030231,3.45359612,3.36235775,3.26749883,3.16996714,3.0707372,2.97080048,2.87115551,2.77279791,2.67671043,2.58385316,2.4951539,2.41149888,2.33372398,2.26260628,2.19885638,2.14311125,2.09592786,2.05777766,2.02904183,2.0100075,2.0]
#goal_x=[5.0,5.09983342,5.19866933,5.29552021,5.38941834,5.47942554,5.56464247,5.64421769,5.71735609,5.78332691,5.84147098,5.89120736,5.93203909,5.96355819,5.98544973,5.99749499,5.9995736,5.99166481,5.97384763,5.94630009,5.90929743,5.86320937,5.8084964,5.74570521,5.67546318,5.59847214,5.51550137,5.42737988,5.33498815,5.23924933,5.14112001,5.0,-5.0,-5.09983342,-5.19866933,-5.29552021,-5.38941834,-5.47942554,-5.56464247,-5.64421769,-5.71735609,-5.78332691,-5.84147098,-5.89120736,-5.93203909,-5.96355819,-5.98544973,-5.99749499,-5.9995736,-5.99166481,-5.97384763,-5.94630009,-5.90929743,-5.86320937,-5.8084964,-5.74570521,-5.67546318,-5.59847214,-5.51550137,-5.42737988,-5.33498815,-5.23924933,-5.14112001,-5.0,5.0,5.09983342,5.19866933,5.29552021,5.38941834,5.47942554,5.56464247,5.64421769,5.71735609,5.78332691,5.84147098,5.89120736,5.93203909,5.96355819,5.98544973,5.99749499,5.9995736,5.99166481,5.97384763,5.94630009,5.90929743,5.86320937,5.8084964,5.74570521,5.67546318,5.59847214,5.51550137,5.42737988,5.33498815,5.23924933,5.14112001,5.0,-5.0,-5.09983342,-5.19866933,-5.29552021,-5.38941834,-5.47942554,-5.56464247,-5.64421769,-5.71735609,-5.78332691,-5.84147098,-5.89120736,-5.93203909,-5.96355819,-5.98544973,-5.99749499,-5.9995736,-5.99166481,-5.97384763,-5.94630009,-5.90929743,-5.86320937,-5.8084964,-5.74570521,-5.67546318,-5.59847214,-5.51550137,-5.42737988,-5.33498815,-5.23924933,-5.14112001,-5.0,5.0]

#goal_y=[4.0,3.99500417,3.98006658,3.95533649,3.92106099,3.87758256,3.82533561,3.76484219,3.69670671,3.62160997,3.54030231,3.45359612,3.36235775,3.26749883,3.16996714,3.0707372,2.97080048,2.87115551,2.77279791,2.67671043,2.58385316,2.4951539,2.41149888,2.33372398,2.26260628,2.19885638,2.14311125,2.09592786,2.05777766,2.02904183,2.0100075,2.0,2.0,1.99500417,1.98006658,1.95533649,1.92106099,1.87758256,1.82533561,1.76484219,1.69670671,1.62160997,1.54030231,1.45359612,1.36235775,1.26749883,1.16996714,1.0707372,0.97080048,0.87115551,0.77279791,0.67671043,0.58385316,0.4951539,0.41149888,0.33372398,0.26260628,0.19885638,0.14311125,0.09592786,0.05777766,0.02904183,0.0100075,0.0,0.0,-0.00499583,-0.01993342,-0.04466351,-0.07893901,-0.12241744,-0.17466439,-0.23515781,-0.30329329,-0.37839003,-0.45969769,-0.54640388,-0.63764225,-0.73250117,-0.83003286,-0.9292628,-1.02919952,-1.12884449,-1.22720209,-1.32328957,-1.41614684,-1.5048461,-1.58850112,-1.66627602,-1.73739372,-1.80114362,-1.85688875,-1.90407214,-1.94222234,-1.97095817,-1.9899925,-2.0,-2.0,-2.00499583,-2.01993342,-2.04466351,-2.07893901,-2.12241744,-2.17466439,-2.23515781,-2.30329329,-2.37839003,-2.45969769,-2.54640388,-2.63764225,-2.73250117,-2.83003286,-2.9292628,-3.02919952,-3.12884449,-3.22720209,-3.32328957,-3.41614684,-3.5048461,-3.58850112,-3.66627602,-3.73739372,-3.80114362,-3.85688875,-3.90407214,-3.94222234,-3.97095817,-3.9899925,-4.0,-4.0]

def animate(i):
    plt.cla()
    #plt.plot(x)
    plt.plot(pos.x, pos.y, label='waypoints')
    plt.legend()


def controller(x_goal, y_goal):
    global theta, pos, kp_omega, kp_lin
    
    # angular vel
    theta_d = np.arctan2((y_goal - pos.y), (x_goal - pos.x))
    theta_e = theta_d - theta 
    theta_e = np.arctan2(np.sin(theta_e),np.cos(theta_e))
    omega = kp_omega * theta_e
    omega = np.clip(omega, -1*Wmax, Wmax)
    

    # linear vel
    d = math.sqrt(pow((x_goal-pos.x),2) + pow((y_goal-pos.y),2))
    v_lin = kp_lin * d
    print(theta_e)
    #v_lin = v_lin*abs(math.cos(theta_e))
    v_lin = np.clip(v_lin, -1*Vmax, Vmax)
    #v_lin = 0.5

    #Linear Controller Equations
    #control_velocity = (vref) + ((c1*xe)/(math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))))   # Controlled Linear Velocity
    #controlled_omega = omega_ref + 0.09 * theta_e + 0.0153 * integral_error + (((c2vref)(yemath.cos(theta_e/2) - xemath.sin(theta_e/2) ))/( math.sqrt( 1 + (pow(xe,2)) + (pow(ye,2)))) ) + c3*math.sin(theta_e/2) # Controlled Angular Velocity
    
    error_previous = theta_e

    
    return v_lin, omega

def callback_pose(data):
    global pos, theta, d_rob2goal
    # Gets current position and orientation (Quaternion)
    pos.x = data.pose.pose.position.x
    pos.y = data.pose.pose.position.y
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]
    #print("pos_x: ,pos_y: ",pos.x,pos.y)
    #print (pos.x, pos.y, theta)

def show_plot():
    ani = FuncAnimation(plt.gcf(), animate, interval = 1000,cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    

def run():
    time_current = time.time()
    print("time_current: ",time_current)
    global pos, goal_x, goal_y, theta
    x_goal = 5.0
    y_goal = 4.0
    break_loop = False
    ang_loop = False
    rospy.init_node('husky_nav', anonymous=True)
    twist = gmsg.Twist()
    #pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', gmsg.Twist, queue_size=1)
    pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)
    pos_sub = rospy.Subscriber('/odom', Odometry, callback_pose)
    rate = rospy.Rate(30)
    rospy.sleep (1)
    while not rospy.is_shutdown():
        
        if break_loop == False:
            for r in range(len(goal_x)):
                if r == len(goal_x)-1:
                    break_loop = True
                x_goal = goal_x[r]
                y_goal = goal_y[r]
                print (r, x_goal, y_goal, "\n\n\n")
                while (round(math.sqrt(pow((x_goal-pos.x),2) + pow((y_goal-pos.y),2)), 2) >= 0.1):
                    #print ("here", x_goal, y_goal, round(math.sqrt(pow((x_goal-pos.x),2) + pow((y_goal-pos.y),2))))
                    v, w = controller(x_goal, y_goal)
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                #rospy.sleep (0.1) ################
                print (r, round(math.sqrt(pow((x_goal-pos.x),2) + pow((y_goal-pos.y),2)), 2))
                rospy.sleep (0.1)
        rate.sleep()
        time_previous = time_current
        print("time_previous: ",time_previous)
if __name__ == '__main__':
    try:
        run()
        #show_plot()
    except rospy.ROSInterruptException:
        pass