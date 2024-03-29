import rospy
from geometry_msgs.msg import TwistStamped,PoseStamped
from ros_clients.msg import GeneralizedForce
import math

class Tekne:
    def __init__(self,istenen_x,istenen_y):
        rospy.init_node("burak",anonymous=True)
        rospy.Subscriber=("/nav/twist",TwistStamped,self.pose_callback)
        self.pub=rospy.Publisher("/force_control",GeneralizedForce,queue_size=10)
        self.x1=istenen_x
        self.y1=istenen_y
        self.integral_x=0
        self.integral_y=0
        self.integral_yaw=0
        self.lasterror_x=0
        self.lasterror_y=0
        self.lasterror_yaw=0
        self.dt=0.1
        self.Kp=0
        self.Ki=0
        self.Kd=0
        self.rate=rospy.Rate(10)
    def pose_callback(self,data:PoseStamped):
        self.x=data.pose.position.x
        self.y=data.pose.position.y
        self.yaw=data.pose.position.z
    def pid_control(self,error,last_error,integral):
        proportional=self.Kp*error
        integral+=error*self.dt
        derivative=self.Kd*(error-last_error)/self.dt
        output=proportional+self.Ki*integral+derivative
        last_error=error
        return output, last_error, integral
    def goal(self):
        force_msg=GeneralizedForce()
        while not rospy.is_shutdown():
            distance=math.sqrt(((self.x1-self.x)**2)+((self.y1-self.y)**2))
            error_yaw=math.atan2(self.y1-self.y,self.x1-self.x)
            force_x, self.lasterror_x, self.integral_x=self.pid_control(distance, self.lasterror_x,self.integral_x)
            force_yaw, self.lasterror_yaw, self.integral_yaw=self.pid_control(error_yaw, self.lasterror_yaw,self.integral_yaw)
            force_msg.x=force_x
            #force_msg.y=force_y
            #force_msg.z=force_yaw
            self.pub.publish(force_msg)
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        tekne=Tekne(1,1,1)
        tekne.goal()
    except rospy.ROSInterruptException:
        pass






