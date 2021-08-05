#!/usr/bin/env python
import math
import numpy
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

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
    rospy.wait_for_service('/gazebo/set_link_state')
    rospy.wait_for_service('/gazebo/get_link_state')
    rospy.wait_for_service('/gazebo/set_physics_properties')
    rospy.sleep(5.0)
    clt_set_real_p = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    clt_get_real_p = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    req_set_real_p = SetLinkStateRequest()
    req_get_real_p = GetLinkStateRequest()
    req_get_real_p.link_name = 'base_link'
    resp_real_p = clt_get_real_p(req_get_real_p)
    real_x = resp_real_p.link_state.pose.position.x
    real_y = resp_real_p.link_state.pose.position.y
    real_a = math.atan2(resp_real_p.link_state.pose.orientation.z, resp_real_p.link_state.pose.orientation.w)*2
    delta_t = 0.001
    print("Gazebo real initial position: " + str([real_x, real_y, real_a]))

    global vx, vy, w
    vx = 0
    vy = 0
    w  = 0
    req_set_real_p.link_state.link_name = 'base_link'
    req_set_real_p.link_state.reference_frame = 'world'
    req_set_real_p.link_state.pose  = Pose()
    req_set_real_p.link_state.twist = Twist()

    br = tf.TransformBroadcaster()
    odom_x = 0
    odom_y = 0
    odom_a = 0
    while not rospy.is_shutdown():
        last_real_x = real_x
        last_real_y = real_y
        last_real_a = real_a
        for i in range(50):
            real_x += vx*math.cos(real_a) * delta_t
            real_y += vx*math.sin(real_a) * delta_t
            real_a += w  * delta_t
            odom_x += vx*math.cos(odom_a) * delta_t
            odom_y += vx*math.sin(odom_a) * delta_t
            odom_a += w  * delta_t
        if real_a > math.pi:
            real_a -= 2*math.pi
        if real_a <= -math.pi:
            real_a += 2*math.pi
        if odom_a > math.pi:
            odom_a -= 2*math.pi
        if odom_a <= -math.pi:
            odom_a += 2*math.pi
        delta_x = real_x - last_real_x
        delta_y = real_y - last_real_y
        delta_a = real_a - last_real_a
        if delta_a  > math.pi:
            delta_a -= 2*math.pi
        if delta_a <= -math.pi:
            delta_a += 2*math.pi
        req_set_real_p.link_state.pose.position.x    = real_x
        req_set_real_p.link_state.pose.position.y    = real_y
        req_set_real_p.link_state.pose.orientation.w = math.cos(real_a/2)
        req_set_real_p.link_state.pose.orientation.z = math.sin(real_a/2)
        if delta_x != 0 or delta_y != 0 or delta_a != 0:
            clt_set_real_p(req_set_real_p)
            """
            odom_x += numpy.random.uniform(-delta_x*0.05, delta_x*0.05)
            odom_y += numpy.random.uniform(-delta_y*0.05, delta_y*0.05)
            odom_a += numpy.random.uniform(-delta_a*0.05, delta_a*0.05)
            if odom_a >  math.pi:
               odom_a -= 2*math.pi
            if odom_a < -math.pi:
               odom_a += 2*math.pi
            """
        br.sendTransform((odom_x, odom_y, 0), tf.transformations.quaternion_from_euler(0, 0, odom_a),
                         rospy.Time.now(), "base_link", "odom")
        br.sendTransform((odom_x, odom_y, 0), tf.transformations.quaternion_from_euler(0, 0, odom_a),
                         rospy.Time.now(), "base_footprint", "odom") 
        loop.sleep()

if __name__ == "__main__":
    main()
