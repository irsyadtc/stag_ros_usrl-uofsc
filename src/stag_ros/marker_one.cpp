#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>


int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker");
  ros::NodeHandle n("~");
  ros::Rate r(1);
  
  
  //tf2 static
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::string parent_frame = "map";
  
  std::string child_frame_id;

  double linear_x, linear_y, linear_z;
  double angular_x, angular_y, angular_z;
  
  //upload value
  n.param<std::string>("child_frame_id", child_frame_id, "STag_bundle");
  n.param<double>("position_x", linear_x, 1.0);
  n.param<double>("position_y", linear_y, 0.0);
  n.param<double>("position_z", linear_z, 1.5);
  n.param<double>("roll", angular_x, -1*M_PI_2);
  n.param<double>("pitch", angular_y, 0.0);
  n.param<double>("yaw", angular_z, 0.0);


  while (ros::ok())
  {
 
		//tf2 ARRAY
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = parent_frame;
		static_transformStamped.child_frame_id = child_frame_id;
		static_transformStamped.transform.translation.x = linear_x;
		static_transformStamped.transform.translation.y = linear_y;
		static_transformStamped.transform.translation.z = linear_z;
		tf2::Quaternion quat;
		quat.setRPY(angular_x, angular_y, angular_z);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
  
    r.sleep();
  }
}
