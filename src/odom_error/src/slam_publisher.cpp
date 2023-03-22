#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main (int argc, char** argv) {
	ros::init(argc, argv, "pose_publisher");
	ros::NodeHandle nh;
	ros::Publisher pose_pub=nh.advertise<geometry_msgs::PointStamped>("base_pose",1000);
	
	//Declare the listener to use c++ tf API
	tf::TransformListener listener;
	//Declare the tranfsorm object to store tf data
	tf::StampedTransform transform;
	geometry_msgs::PointStamped msg;
	msg.header.frame_id="/map";
	

	

	ros::Time start = ros::Time::now();
	
	//TO DO: dopo un certo tempo pubblicare il risultato corretto dello slam
	
	while(ros::ok()){
		ros::Time now = ros::Time::now();
		
		if( listener.waitForTransform("/map", "/base_footprint", now, ros::Duration(0.5)) ) 
			listener.lookupTransform("/map", "/base_footprint",  now, transform);
		else	
			continue;
			
		msg.point.x=transform.getOrigin().getX();
		msg.point.y=transform.getOrigin().getY();
		msg.point.z=transform.getOrigin().getZ();
		msg.header.stamp=now;
		
		pose_pub.publish(msg);
		}
		
	
	
	
	}
