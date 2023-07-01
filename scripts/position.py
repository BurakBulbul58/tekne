import rospy
from ros_clients.msg import GeneralizedForce
from geometry_msgs.msg import TwistStamped


class Tekne():
    def __init__(self,Kp,Kd,Ki,istenen_x,istenen_y,istenen_z):
        self.force_publisher=force_publisher
        self.force=GeneralizedForce()
        self.Setpoint_x=istenen_x#radian to kilometer yapmadım henüz
        self.Setpoint_x=istenen_x
        self.Setpoint_x=istenen_x
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.delta_time=0.1

if __name__=="__main__":
    try: 
        rospy.init_node("burak",anonymous=True)
        force_publisher=rospy.Publisher("/force_control",GeneralizedForce,queue_size=10)
        tekne=Tekne(force_publisher,-1.0,1.0,1.0,1.0,1.0,1.0)
        velocity_subscriber=rospy.Subscriber("/nav/twist",TwistStamped,tekne.velocity_callback)
        tekne.update()
    except:
        pass