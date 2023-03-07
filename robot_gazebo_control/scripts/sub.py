import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped

wheel_diameter = 0.2

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")

def listener():
    rospy.init_node('opensg_robot', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()




# <cpp.file sub> 1
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
# void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
# { 
#     ROS_INFO("I heard: [%f]", vel_cmd.linear.y);
#     std::cout << "Twist Received " << std::endl;
# }


# int main( int argc, char* argv[] )
# {
# ros::init(argc, argv, "toeminator_publisher" );

# ros::NodeHandle n;
# ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmd_vel_callback);

# while( n.ok() ) 
# {
#     ros::spin();
# }

# return 1;
# }

# <cpp.file sub> 2
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <geometry_msgs/Twist.h>

# For moving the robot on the map
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "Consts.h" 
#include "RobotInterface.h"
#include "NetworkInterface.h"

# void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
# { 
# ROS_INFO("I heard: [%s]", vel_cmd.linear.y);
# cout << "Twist Received " << endl;
# }

# int main( int argc, char* argv[] )
# {
# ros::init(argc, argv, "opensg_robot" );

# ros::NodeHandle n_("~");
# ros::NodeHandle n("");
# ros::NodeHandle lSubscriber("");
# ros::NodeHandle nIR("");
# ros::NodeHandle nLaser("");
# ros::Rate loop_rate(10);
#     ros::Subscriber sub = lSubscriber.subscribe("/cmd_vel", 1000, cmd_vel_callback);

# ros::Time current_time, last_time;
# current_time = ros::Time::now();
# last_time = ros::Time::now();

# RobotData lRobotData;

# while( n.ok() ) 
# {
#     ros::spin();
# }

# return 1;
# }