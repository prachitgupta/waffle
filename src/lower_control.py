#!/usr/bin/env python3

import rospy
from waffle.msg import Tupla
from geometry_msgs.msg import Twist
import threading

class PID:

    def __init__(self,P=0.0,I=0.0,D=0.0,Derivator=0,Integrator=0,Integrator_max=10,Integrator_min=-10):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.error=0

    def update(self,error):

        self.P_value=self.Kp*error

        self.D_value=self.Kd*(error-self.Derivator) ##basically previous error
        self.Derivator=error

        self.Integrator=self.Integrator+error ##sum
        ##cap integral
        if self.Integrator>self.Integrator_max:
            self.Integrator=self.Integrator_max
        elif self.Integrator<self.Integrator_min:
            self.Integrator=self.Integrator_min
            
        self.I_value=self.Integrator*self.Ki

        PID=self.P_value+self.I_value+self.D_value
        
        return PID

    def setPID(self,set_P=0.0,set_I=0.0,set_D=0.0):

        self.Kp=set_P
        self.Ki=set_I
        self.Kd=set_D

class Control(threading.Thread):

    def __init__(self) -> None:
        
        self.w_depth=320
        self.h_depth=240

        self.delta_pid=PID(P=8,I=0.06,D=0.15)
        self.max_ang_vel=0.2
        self.max_linear_vel=0.125

        self.f=0
        self.dc=0

        threading.Thread.__init__(self)
        self.pub_cmd=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.vel_msg=Twist()

        rospy.Subscriber("/control_value",Tupla,self.controller_cb)
        rospy.on_shutdown(self.stop)

    def controller_cb(self,data):
        
        self.f=data.flag
        self.dc=data.control

    def move(self,x,theta):

        self.vel_msg.linear.x=x
        self.vel_msg.angular.z=theta
        self.pub_cmd.publish(self.vel_msg)
    ##main point of interest
    def depth_controller(self,delta):

        if delta>0: ##error
            ang_vel=float(-self.delta_pid.update(delta)*self.delta_pid.update(delta))/((self.w_depth/2)*(self.w_depth/2))
            
        else:
            ang_vel=float(-self.delta_pid.update(delta)*self.delta_pid.update(delta))/((self.w_depth/2)*(self.w_depth/2))
        
        lin_vel=float(self.max_linear_vel*(1-((delta*delta)/((self.w_depth/2)*(self.w_depth/2)))))

        if abs(ang_vel)>self.max_ang_vel:
            ang_vel=ang_vel/abs(ang_vel)*self.max_ang_vel

        return lin_vel,ang_vel
    
    def stop(self):

        self.move(0,0)
    
if __name__=="__main__":

    rospy.init_node("lower_control")

    c=Control()

    while not rospy.is_shutdown():

        try:

            if not c.f: ##depth flag 0
                
                (cmd_lin2,cmd_ang2)=c.depth_controller(c.dc)
                print(cmd_lin2,cmd_ang2)
                c.move(cmd_lin2,cmd_ang2)

            else:

                if c.dc==0:
                    c.move(0,-0.1)
                
                elif c.dc==1:
                    c.move(0.0,0)

                elif c.dc==2:
                    c.move(0,0.1)

                else:
                    c.move(0,0)

        except rospy.ROSInterruptException:
            rospy.loginfo("Mission aborted")
