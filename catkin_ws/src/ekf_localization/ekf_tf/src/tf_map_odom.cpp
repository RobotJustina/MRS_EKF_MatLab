#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


nav_msgs::Odometry ROBOT_ODOM;
geometry_msgs::PoseWithCovariance  LOCALIZATION_POSE;
geometry_msgs::TransformStamped tf_map_to_odom_;
 
double normalize(double z)
{
  return atan2(sin(z),cos(z));
}


double getYawFromQuaternion(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw; 
	tf2::Quaternion quat_tf;
	tf2::fromMsg(quat, quat_tf);
	tf2::Matrix3x3 m( quat_tf );
	m.getRPY(roll, pitch, yaw);
	//yaw = atan2( (float)yaw );
	return normalize(yaw);
}

void OdomCallback(const nav_msgs::Odometry& msg)
{
	ROBOT_ODOM = msg ;
}

void LocalizationEkfCallback(const geometry_msgs::PoseWithCovariance& msg)
{
    LOCALIZATION_POSE = msg;


   

    //std::cout << getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation  )<< "\n";
    std::cout << "XXXXXXXX"<< normalize( normalize(0.7) - normalize(-0.7) ) << "\n";
    std::cout << "XXXXXXXX First "<< getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation ) << "\n";
    std::cout << "XXXXXXXX Second "<< getYawFromQuaternion( ROBOT_ODOM.pose.pose.orientation )<< "\n";
    std::cout << "XXXXXXXX Res "<< getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation ) - getYawFromQuaternion( ROBOT_ODOM.pose.pose.orientation )  << "\n";
    tf2::Quaternion q;
    q.setRPY(0, 0, normalize( normalize(getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation )) - normalize(getYawFromQuaternion( ROBOT_ODOM.pose.pose.orientation )) ) );
    q.normalize();
    tf_map_to_odom_.transform.rotation.x = q.x();
    tf_map_to_odom_.transform.rotation.y = q.y();
    tf_map_to_odom_.transform.rotation.z = q.z();
    tf_map_to_odom_.transform.rotation.w = q.w();


    double theta = normalize( normalize(getYawFromQuaternion( LOCALIZATION_POSE.pose.orientation )) - normalize(getYawFromQuaternion( ROBOT_ODOM.pose.pose.orientation )) ) ;

    tf_map_to_odom_.transform.translation.x =  LOCALIZATION_POSE.pose.position.x - ROBOT_ODOM.pose.pose.position.x*cos(theta) + ROBOT_ODOM.pose.pose.position.y*sin(theta);
    tf_map_to_odom_.transform.translation.y =  LOCALIZATION_POSE.pose.position.y - ROBOT_ODOM.pose.pose.position.x*sin(theta) - ROBOT_ODOM.pose.pose.position.y*cos(theta);
    tf_map_to_odom_.transform.translation.z = 0;

}




int main(int argc, char *argv[])
{

	ros::init(argc, argv, "Nodo_ekf_tf");
	ros::NodeHandle nh;
    ros::Subscriber localization_ekf_sub = nh.subscribe("localization_ekf", 0, LocalizationEkfCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 0, OdomCallback);

    int32_t publish_rate_ = 100;
    tf2_ros::TransformBroadcaster tf_br_;

    double initial_pose_x;
    double initial_pose_y;
    double initial_pose_a;


    ros::param::get("~initial_pose_x", initial_pose_x);
    ros::param::get("~initial_pose_y", initial_pose_y);
    ros::param::get("~initial_pose_a", initial_pose_a);

    tf2::Quaternion q;
    q.setRPY(0, 0, normalize( initial_pose_a ) );
    q.normalize();

    tf_map_to_odom_.transform.translation.x = initial_pose_x;
    tf_map_to_odom_.transform.translation.y = initial_pose_y;
    tf_map_to_odom_.transform.translation.z = 0;
    

    tf_map_to_odom_.transform.rotation.x = q.x();
    tf_map_to_odom_.transform.rotation.y = q.y();
    tf_map_to_odom_.transform.rotation.z = q.z();
    tf_map_to_odom_.transform.rotation.w = q.w();



    // set up parent and child frames
    tf_map_to_odom_.header.frame_id = std::string("map");
    tf_map_to_odom_.child_frame_id = std::string("odom");

    // set up publish rate
    
    ros::Rate loop_rate(publish_rate_);
    std::cout << "Waiting for kalman pose..." << std::endl;
    while(localization_ekf_sub.getNumPublishers() == 0)
	{
		loop_rate.sleep();
        ros::spinOnce();
	}
    std::cout << "Waiting for Odom pose..." << std::endl;
    while(odom_sub.getNumPublishers() == 0)
	{
		loop_rate.sleep();
        ros::spinOnce();
	}
	std::cout << "OK." << std::endl;

    double incremento = 0;
    // main loop
    while (ros::ok())
    {
        // time stamp
        tf_map_to_odom_.header.stamp = ros::Time::now();

        // specify actual transformation vectors from odometry
        // NOTE: zeros have to be substituted with actual variable data
        

        incremento += .01;
        
        
        // broadcast transform
        tf_br_.sendTransform(tf_map_to_odom_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
