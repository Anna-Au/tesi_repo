#include <iostream>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>

#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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

std::mutex m_table_mutex;

int number=1;


void map_callback(const aruco_msgs::MarkerArray::ConstPtr& marker_msg){
	bool present=false;
	aruco_msgs::Marker marker;
	//ros::Rate r(5);
	
	ROS_INFO("Map Marker Cb executing");
	
	if(m_table_mutex.try_lock()){
	//for each marker detected by the camera, check if it is already present in the table.
	//if present update the position and transform, otherwise add it to the table
	for(auto it = marker_msg->markers.begin(); it != marker_msg->markers.end(); ++it){
		marker=*it;
		tf::Vector3 marker_pos=tf::Vector3(marker.pose.pose.position.x, marker.pose.pose.position.y,marker.pose.pose.position.z);
		tf::Quaternion marker_rot=tf::Quaternion(marker.pose.pose.orientation.x,marker.pose.pose.orientation.y,marker.pose.pose.orientation.z,marker.pose.pose.orientation.w);
		
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
	
	
		if(min_distance<2){
			
			(*min_dist_it).pos=marker_pos;
			tf::Transform new_transform;
			new_transform.setOrigin(marker_pos);
  			new_transform.setRotation(marker_rot);
  			(*min_dist_it).marker2map=new_transform;
			}
		else{
		std::cout<<"adding marker2map transform to table"<<std::endl;
		obs_marker new_marker;
		new_marker.pos=marker_pos;
		new_marker.id=marker.id;
		new_marker.num=number;
		number=number+1;
		tf::Transform new_transform;
		new_transform.setOrigin(marker_pos);
	  	new_transform.setRotation(marker_rot);
	  	
	  	new_marker.marker2map=new_transform;
		marker_table.push_back(new_marker);
	
		}
	
	}
	m_table_mutex.unlock();
	
	}
	
	//r.sleep();


}


/*void camera_callback(const aruco_msgs::MarkerArray::ConstPtr& marker_msg){
	bool present=false;
	aruco_msgs::Marker marker;
	
	std::cout<<"Camera Marker Cb executing"<<std::endl;
	
	//save timestamp
	ros::Time time_stamp=marker_msg->header.stamp;
	
	for(auto it = marker_msg->markers.begin(); it != marker_msg->markers.end(); ++it){
		marker=*it;
		//std::cout<<"marker"<< marker.id << std::endl;
		//std::cout<<"position"<< marker.pose.pose.position.x << " " << marker.pose.pose.position.y <<" " <<marker.pose.pose.position.z << std::endl;
		break;
		}
		
	for(auto it2 = table.begin(); it2 != table.end(); ++it2){
		if (marker.id==(*it2).id){
		//correct position if the marker is already present in table and exit 
		tf::Transform marker2map = (*it2).marker2map;
		
		
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
	  	break;
		} 	
		
	}
          
          sleep(5);
}*/




int main (int argc, char** argv){
	ros::init(argc,argv,"marker_correction");
	ros::NodeHandle n;
	//ros::Subscriber marker_sub=n.subscribe("aruco_marker_publisher/markers",100,callback);
	
	
	ros::Subscriber map_marker_sub=n.subscribe("map_marker/markers",1,map_callback);
	
	tf::TransformBroadcaster br;
	
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	//ros::waitForShutdown();
	ros::Rate r(5);
	
	
	while(ros::ok()){
		ROS_INFO("Main executing");
		
		
		m_table_mutex.lock();
		for(auto it2 = marker_table.begin(); it2 != marker_table.end(); ++it2){
			std::string str="/marker_";
	  		str += std::to_string((*it2).num);
			
			br.sendTransform(tf::StampedTransform((*it2).marker2map,ros::Time::now(),"/map",str));
			std::cout<<"Publishing transform from: /map to: "<< str << std::endl;
			}
		m_table_mutex.unlock();
		r.sleep();
		
		
		
		}	
	
	

}
