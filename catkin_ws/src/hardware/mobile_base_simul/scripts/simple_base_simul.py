#!/usr/bin/env python
import math
import numpy
import rospy
import tf
from geometry_msgs.msg import Twist, Pose

def callback_cmd_vel(msg):
    global vx, vy, w
    vx = msg.linear.x
    vy = msg.linear.y
    w  = msg.angular.z

def main():
    print("INITIALIZING MOBILE BASE ROS-GAZEBO BRIDGE...")
    rospy.init_node("mobile_base")
    rospy.Subscriber('/cmd_vel', Twist, callback_cmd_vel)
    loop = rospy.Rate(20)
    
    global vx, vy, w
    vx = 0
    vy = 0
    w  = 0
    br = tf.TransformBroadcaster()
    odom_x = 0
    odom_y = 0
    odom_a = 0
    delta_t = 0.001
    while not rospy.is_shutdown():
        for i in range(50):
            odom_x += vx*math.cos(odom_a) * delta_t
            odom_y += vx*math.sin(odom_a) * delta_t
            odom_a += w  * delta_t
        if odom_a > math.pi:
            odom_a -= 2*math.pi
        if odom_a <= -math.pi:
            odom_a += 2*math.pi
        br.sendTransform((odom_x, odom_y, 0), tf.transformations.quaternion_from_euler(0, 0, odom_a),
                         rospy.Time.now(), "base_link", "odom") 
        loop.sleep()

if __name__ == "__main__":
    main()
