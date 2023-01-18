#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#include "include/markersingle.hpp"
#include "include/markersingle.cpp"


int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  
  //tf2 static
  static tf2_ros::StaticTransformBroadcaster static_broadcaster[NO_OF_MARKERS];
  geometry_msgs::TransformStamped static_transformStamped[NO_OF_MARKERS];
  std::string parent_frame = "map";
  markersingle MarkerSingle;
  

  while (ros::ok())
  {
 
		//tf2 ARRAY
		for(unsigned int i=0; i<NO_OF_MARKERS; i++)
		{
			static_transformStamped[i].header.stamp = ros::Time::now();
  		static_transformStamped[i].header.frame_id = parent_frame;
  		static_transformStamped[i].child_frame_id = MarkerSingle.getName(i);
  		static_transformStamped[i].transform.translation.x = MarkerSingle.getPosx(i);
  		static_transformStamped[i].transform.translation.y = MarkerSingle.getPosy(i);
  		static_transformStamped[i].transform.translation.z = MarkerSingle.getPosz(i);
  		tf2::Quaternion quat;
  		quat.setRPY(MarkerSingle.getRoll(i), MarkerSingle.getPitch(i), MarkerSingle.getYaw(i));
  		static_transformStamped[i].transform.rotation.x = quat.x();
  		static_transformStamped[i].transform.rotation.y = quat.y();
  		static_transformStamped[i].transform.rotation.z = quat.z();
  		static_transformStamped[i].transform.rotation.w = quat.w();
			static_broadcaster[i].sendTransform(static_transformStamped[i]);
	
		} 
  
    r.sleep();
  }
}
