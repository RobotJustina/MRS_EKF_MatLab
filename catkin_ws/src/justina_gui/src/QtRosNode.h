#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "tf/transform_listener.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubCmdVel;
    ros::Publisher pubTorso;
    ros::Publisher pubLaAngle1;
    ros::Publisher pubLaAngle2;
    ros::Publisher pubLaAngle3;
    ros::Publisher pubLaAngle4;
    ros::Publisher pubLaAngle5;
    ros::Publisher pubLaAngle6;
    ros::Publisher pubLaAngle7;
    ros::Publisher pubLaAngleGl;
    ros::Publisher pubLaAngleGr;
    ros::Publisher pubRaAngle1;
    ros::Publisher pubRaAngle2;
    ros::Publisher pubRaAngle3;
    ros::Publisher pubRaAngle4;
    ros::Publisher pubRaAngle5;
    ros::Publisher pubRaAngle6;
    ros::Publisher pubRaAngle7;
    ros::Publisher pubRaAngleGl;
    ros::Publisher pubRaAngleGr;
    ros::Publisher pubHdPan;
    ros::Publisher pubHdTilt;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);

    void publish_torso_position(float tr);
    void publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_la_grip_angles(float a1, float a2);
    void publish_ra_grip_angles(float a1, float a2);
    void publish_head_angles(float pan, float tilt);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
