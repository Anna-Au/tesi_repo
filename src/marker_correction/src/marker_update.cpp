#include <iostream>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>

#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <limits>


typedef struct{
	int id;
	int num;
 	tf::Vector3 pos;
 	tf::Transform marker2map;

}obs_marker;

std::vector<obs_marker> marker_table;
ros::Publisher pose_pub;


void camera_callback(const aruco_msgs::MarkerArray::ConstPtr& marker_msg){
	bool present=false;
	aruco_msgs::Marker marker;
	
	std::cout<<"Camera Marker Cb executing"<<std::endl;
	
	//save timestamp
	ros::Time time_stamp=marker_msg->header.stamp;
	
	for(auto it = marker_msg->markers.begin(); it != marker_msg->markers.end(); ++it){
		marker=*it;
		tf::Vector3 marker_pos=tf::Vector3(marker.pose.pose.position.x, marker.pose.pose.position.y,marker.pose.pose.position.z);
		//std::cout<<"marker"<< marker.id << std::endl;
		//std::cout<<"position"<< marker.pose.pose.position.x << " " << marker.pose.pose.position.y <<" " <<marker.pose.pose.position.z << std::endl;
		double min_distance = std::numeric_limits<double>::infinity();
		std::vector<obs_marker>::iterator min_dist_it;
		
		//search the element in the table with same id that minimizes the distance
		for(auto it2 = marker_table.begin(); it2 != marker_table.end(); ++it2){
			if(marker.id == (*it2).id){
				std::cout<<"Distance : " << marker_pos.distance((*it2).pos) <<std::endl;
				if (marker_pos.distance((*it2).pos)<min_distance ){
				//check if the marker is already present, otherwise add it to the table 
					
					min_distance=marker_pos.distance((*it2).pos);
					min_dist_it=it2;
				} 
			}
		}
		
		
		
		
		tf::Transform marker2map = (*min_dist_it).marker2map;
		
		
		
		
		
		
		tf::Transform marker2base_link;
  		marker2base_link.setOrigin( tf::Vector3(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z) );
  		marker2base_link.setRotation(tf::Quaternion(marker.pose.pose.orientation.w, 						marker.pose.pose.orientation.x,marker.pose.pose.orientation.y,marker.pose.pose.orientation.z) );
  					
  				
  		
  		
	        //generate a pose_stamped that will update amcl
	       
	        tf::Transform base_link2map = marker2base_link.inverse() * marker2map;
	        
  		static tf::TransformBroadcaster br;	
	  	br.sendTransform(tf::StampedTransform(base_link2map, time_stamp, "/base_link", "/map"));
	  	geometry_msgs::PoseWithCovarianceStamped init_pose_msg;
	  	init_pose_msg.pose.pose.position.x=base_link2map.getOrigin().x();
	  	init_pose_msg.pose.pose.position.y=base_link2map.getOrigin().y();
	  	init_pose_msg.pose.pose.position.z=base_link2map.getOrigin().z();
	  	init_pose_msg.pose.pose.orientation.x=base_link2map.getRotation().x();
	  	init_pose_msg.pose.pose.orientation.y=base_link2map.getRotation().y();
	  	init_pose_msg.pose.pose.orientation.z=base_link2map.getRotation().z();
	  	init_pose_msg.pose.pose.orientation.w=base_link2map.getRotation().w();
	  	init_pose_msg.header.stamp=time_stamp;
	  	pose_pub.publish(init_pose_msg);
	  	std::cout<<"Updating initial pose in amcl"<<std::endl;	
		} 	
		
	}
          


int main (int argc, char** argv){
	ros::init(argc,argv,"marker_update");
	ros::NodeHandle n;
	ros::Publisher pose_pub=n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl/initialpose",10);
	
	
	ros::Subscriber base_marker_sub=n.subscribe("base_marker/markers",1,camera_callback);
	
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	
		
		
	tf::StampedTransform transform;
	obs_marker new_marker;
	try{
	listener.lookupTransform("/map", "/marker_1",  
		         ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	}
	
	new_marker.marker2map=transform;
	new_marker.num=1;
	new_marker.id=60;
	
	try{
	listener.lookupTransform("/map", "/marker_2",  
		         ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	}
	
	new_marker.marker2map=transform;
	new_marker.num=2;
	new_marker.id=100;
	
	try{
	listener.lookupTransform("/map", "/marker_3",  
		         ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	}
	
	new_marker.marker2map=transform;
	new_marker.num=3;
	new_marker.id=60;
	
	
	try{
	listener.lookupTransform("/map", "/marker_4",  
		         ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	}
	
	new_marker.marker2map=transform;
	new_marker.num=4;
	new_marker.id=100;



	ros::spin();
	return 0;
	
		
		
	

}
