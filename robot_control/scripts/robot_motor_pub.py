import rospy
import sys
from geometry_msgs.msg import Twist

def publish_velocity_commands():
    vel_pub =rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('robot_motor_pub', anonymous=True)

    msg=Twist()
    msg.linear.x=0.5
    msg.linear.y=0.5
    msg.linear.z=0.5
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=0

    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        vel_pub.publish(msg)
        rate.sleep()

if __name__ == '_main_':
    if len(sys.argv) == 1:
        try:
            publish_velocity_commands()
        except rospy.ROSInterruptException:
                pass
    else:
        print("booboo")
        
