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
  static tf2_ros::StaticTransformBroadcaster static_broadcaster[2];
  geometry_msgs::TransformStamped static_transformStamped[2];
  std::string parent_frame = "map";
  
  std::string child_frame_id[2];

  double linear_x[2], linear_y[2], linear_z[2];
  double angular_x[2], angular_y[2], angular_z[2];
  
  //upload values
  n.param<std::string>("child_frame_id_1", child_frame_id[0], "STag_tag");
  n.param<double>("position_x_1", linear_x[0], 1.0);
  n.param<double>("position_y_1", linear_y[0], 0.0);
  n.param<double>("position_z_1", linear_z[0], 1.5);
  n.param<double>("roll_1", angular_x[0], -1*M_PI_2);
  n.param<double>("pitch_1", angular_y[0], 0.0);
  n.param<double>("yaw_1", angular_z[0], 0.0);
  
  n.param<std::string>("child_frame_id_2", child_frame_id[1], "STag_tag");
  n.param<double>("position_x_2", linear_x[1], -1.0);
  n.param<double>("position_y_2", linear_y[1], 0.0);
  n.param<double>("position_z_2", linear_z[1], 1.5);
  n.param<double>("roll_2", angular_x[1], -1*M_PI_2);
  n.param<double>("pitch_2", angular_y[1], 0.0);
  n.param<double>("yaw_2", angular_z[1], 0.0);


  while (ros::ok())
  {
 
		//tf2 ARRAY
		static_transformStamped[0].header.stamp = ros::Time::now();
		static_transformStamped[0].header.frame_id = parent_frame;
		static_transformStamped[0].child_frame_id = child_frame_id[0];
		static_transformStamped[0].transform.translation.x = linear_x[0];
		static_transformStamped[0].transform.translation.y = linear_y[0];
		static_transformStamped[0].transform.translation.z = linear_z[0];
		tf2::Quaternion quat[2];
		quat[0].setRPY(angular_x[0], angular_y[0], angular_z[0]);
		static_transformStamped[0].transform.rotation.x = quat[0].x();
		static_transformStamped[0].transform.rotation.y = quat[0].y();
		static_transformStamped[0].transform.rotation.z = quat[0].z();
		static_transformStamped[0].transform.rotation.w = quat[0].w();
		static_broadcaster[0].sendTransform(static_transformStamped[0]);
		
		static_transformStamped[1].header.stamp = ros::Time::now();
		static_transformStamped[1].header.frame_id = parent_frame;
		static_transformStamped[1].child_frame_id = child_frame_id[1];
		static_transformStamped[1].transform.translation.x = linear_x[1];
		static_transformStamped[1].transform.translation.y = linear_y[1];
		static_transformStamped[1].transform.translation.z = linear_z[1];
		//tf2::Quaternion quat;
		quat[1].setRPY(angular_x[1], angular_y[1], angular_z[1]);
		static_transformStamped[1].transform.rotation.x = quat[1].x();
		static_transformStamped[1].transform.rotation.y = quat[1].y();
		static_transformStamped[1].transform.rotation.z = quat[1].z();
		static_transformStamped[1].transform.rotation.w = quat[1].w();
		static_broadcaster[1].sendTransform(static_transformStamped[1]);
  
    r.sleep();
  }
}
