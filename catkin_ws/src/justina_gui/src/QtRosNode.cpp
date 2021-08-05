#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel    = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pubTorso     = n->advertise<std_msgs::Float64>("/torso_controller/command", 10);
    pubLaAngle1  = n->advertise<std_msgs::Float64>("/la_1_controller/command" , 10);
    pubLaAngle2  = n->advertise<std_msgs::Float64>("/la_2_controller/command" , 10);
    pubLaAngle3  = n->advertise<std_msgs::Float64>("/la_3_controller/command" , 10);
    pubLaAngle4  = n->advertise<std_msgs::Float64>("/la_4_controller/command" , 10);
    pubLaAngle5  = n->advertise<std_msgs::Float64>("/la_5_controller/command" , 10);
    pubLaAngle6  = n->advertise<std_msgs::Float64>("/la_6_controller/command" , 10);
    pubLaAngle7  = n->advertise<std_msgs::Float64>("/la_7_controller/command" , 10);
    pubRaAngle1  = n->advertise<std_msgs::Float64>("/ra_1_controller/command" , 10);
    pubRaAngle2  = n->advertise<std_msgs::Float64>("/ra_2_controller/command" , 10);
    pubRaAngle3  = n->advertise<std_msgs::Float64>("/ra_3_controller/command" , 10);
    pubRaAngle4  = n->advertise<std_msgs::Float64>("/ra_4_controller/command" , 10);
    pubRaAngle5  = n->advertise<std_msgs::Float64>("/ra_5_controller/command" , 10);
    pubRaAngle6  = n->advertise<std_msgs::Float64>("/ra_6_controller/command" , 10);
    pubRaAngle7  = n->advertise<std_msgs::Float64>("/ra_7_controller/command" , 10);
    pubLaAngleGl = n->advertise<std_msgs::Float64>("/la_grip_left_controller/command" , 10);
    pubLaAngleGr = n->advertise<std_msgs::Float64>("/la_grip_right_controller/command", 10);
    pubRaAngleGl = n->advertise<std_msgs::Float64>("/ra_grip_left_controller/command" , 10);
    pubRaAngleGr = n->advertise<std_msgs::Float64>("/ra_grip_right_controller/command", 10);
    pubHdPan     = n->advertise<std_msgs::Float64>("/head_pan_controller/command",  10);
    pubHdTilt    = n->advertise<std_msgs::Float64>("/head_tilt_controller/command", 10);

    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

void QtRosNode::publish_torso_position(float tr)
{
    std_msgs::Float64 msg;
    msg.data = tr;
    pubTorso.publish(msg);
}

void QtRosNode::publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64 msg;
    msg.data = a1;
    pubLaAngle1.publish(msg);
    msg.data = a2;
    pubLaAngle2.publish(msg);
    msg.data = a3;
    pubLaAngle3.publish(msg);
    msg.data = a4;
    pubLaAngle4.publish(msg);
    msg.data = a5;
    pubLaAngle5.publish(msg);
    msg.data = a6;
    pubLaAngle6.publish(msg);
    msg.data = a7;
    pubLaAngle7.publish(msg);
}

void QtRosNode::publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64 msg;
    msg.data = a1;
    pubRaAngle1.publish(msg);
    msg.data = a2;
    pubRaAngle2.publish(msg);
    msg.data = a3;
    pubRaAngle3.publish(msg);
    msg.data = a4;
    pubRaAngle4.publish(msg);
    msg.data = a5;
    pubRaAngle5.publish(msg);
    msg.data = a6;
    pubRaAngle6.publish(msg);
    msg.data = a7;
    pubRaAngle7.publish(msg);
}

void QtRosNode::publish_la_grip_angles(float al, float ar)
{
    std_msgs::Float64 msg;
    msg.data = al;
    pubLaAngleGl.publish(msg);
    msg.data = ar;
    pubLaAngleGr.publish(msg);
}

void QtRosNode::publish_ra_grip_angles(float al, float ar)
{
    std_msgs::Float64 msg;
    msg.data = al;
    pubRaAngleGl.publish(msg);
    msg.data = ar;
    pubRaAngleGr.publish(msg);
}

void QtRosNode::publish_head_angles(float pan, float tilt)
{
    std_msgs::Float64 msg;
    msg.data = pan;
    pubHdPan.publish(msg);
    msg.data = tilt;
    pubHdTilt.publish(msg);
}
