#!/usr/bin/env python3
import rospy
from ros_clients.msg import GeneralizedForce
from geometry_msgs.msg import TwistStamped
class Tekne():
    def __init__(self,force_publisher,Desired_velx,Desired_vely,Desired_velz,Kp,Ki,Kd):
        self.force_publisher=force_publisher
        self.force=GeneralizedForce()
        self.SetPoint_x=Desired_velx
        self.SetPoint_y=Desired_vely
        self.Setpoint_z=Desired_velz
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.delta_time=0.1
        self.x=0
        self.y=0
        self.z=0
        self.Pterm1=0.0
        self.Iterm1=0.0
        self.Dterm1=0.0
        self.Pterm2=0.0
        self.Iterm2=0.0
        self.Dterm2=0.0
        self.Pterm3=0.0
        self.Iterm3=0.0
        self.Dterm3=0.0
        self.lasterr1=0.0
        self.lasterr2=0.0
        self.lasterr3=0.0
        self.windup_guard=32.35*Desired_velx
        self.output1=0.0
        self.output2=0.0
        self.output3=0.0
        self.error_x=0.0
        self.error_y=0.0
        self.error_z=0.0
        self.delta_error1=0.0
        self.delta_error2=0.0
        self.delta_error3=0.0
        
    def velocity_callback(self,a:TwistStamped):
        self.x=a.twist.linear.x
        self.y=a.twist.linear.y
        self.z=a.twist.angular.z
        
    def update(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            self.error_x=self.x-self.SetPoint_x
            self.error_y=self.SetPoint_y-self.y
            self.error_z=self.Setpoint_z-self.z
            self.delta_error1=self.error_x-self.lasterr1
            self.delta_error2=self.error_y-self.lasterr2
            self.delta_error3=self.error_z-self.lasterr3
            self.Pterm1 = self.Kp * self.error_x
            self.Iterm1 += self.error_x * self.delta_time
            self.Pterm2 = self.Kp * self.error_y
            self.Iterm2 += self.error_y * self.delta_time
            self.Pterm3 = self.Kpz * self.error_z
            self.Iterm3 += self.error_z * self.delta_time

            # if (self.Iterm1 < -self.windup_guard):
            #     self.Iterm1 = -self.windup_guard
            # elif (self.Iterm1 > self.windup_guard):
            #     self.Iterm1 = self.windup_guard
            # if (self.Iterm2 < -self.windup_guard):
            #     self.Iterm2 = -self.windup_guard
            # elif (self.Iterm2 > self.windup_guard):
            #     self.Iterm2 = self.windup_guard
            # if (self.Iterm3 < -self.windup_guard):
            #     self.Iterm3 = -self.windup_guard
            # elif (self.Iterm3 > self.windup_guard):
            #     self.Iterm3 = self.windup_guard 
            
            self.Dterm1=self.delta_error1/self.delta_time
            self.Dterm2=self.delta_error2/self.delta_time
            self.Dterm3=self.delta_error3/self.delta_time
            self.lasterr1=self.error_x
            self.lasterr2=self.error_y
            self.lasterr3=self.error_z
            
            self.output1 = self.Pterm1 + (self.Ki * self.Iterm1) + (self.Kd * self.Dterm1)
            self.output2 = self.Pterm2 + (self.Ki * self.Iterm2) + (self.Kd * self.Dterm2)
            self.output3 = self.Pterm3 + (self.Ki * self.Iterm3) + (self.Kd * self.Dterm3)
            print("output1:",self.output1)
            print("self.x:",self.x) 
            #print("output2:",self.output2)
            print("output3:",self.output3)
            print("self.z",self.z)
            self.force.x=self.output1
            #self.force.y=self.output2
            self.force.n=self.output3
            self.force_publisher.publish(self.force)
            rate.sleep()

if __name__=="__main__":
    try: 
        rospy.init_node("burak",anonymous=True)
        force_publisher=rospy.Publisher("/force_control",GeneralizedForce,queue_size=10)
        tekne=Tekne(force_publisher,-1.0,1.0,1.0,1.0,1.0,1.0)
        velocity_subscriber=rospy.Subscriber("/nav/twist",TwistStamped,tekne.velocity_callback)
        tekne.update()
    except rospy.ROSInterruptException:
        pass