#include <iostream>
#include <iterator>
#include <random>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main (int argc, char** argv) {
	ros::init(argc, argv, "odom_error");
	ros::NodeHandle nh;
	
	//Declare the listener to use c++ tf API
	tf::TransformListener listener;
	//Declare the tranfsorm object to store tf data
	tf::StampedTransform transform;
	//Declare the boradcaster to use c++ tf API
	tf::TransformBroadcaster br;
	tf::Transform error_transform;
	
	tf::Quaternion q(0,0,0,1);
	
	
    
	// Define random generator with Gaussian distribution
	const double mean = 0.0;
	const double stddev = 0.1;
	std::default_random_engine generator;
	std::normal_distribution<double> dist(mean, stddev);

	ros::Rate rate(1);

	ros::Time start = ros::Time::now();
	
	//TO DO: dopo un certo tempo pubblicare il risultato corretto dello slam
	
	while(ros::ok()){
		ros::Time now = ros::Time::now();
		if(now-start<ros::Duration(36000.0)){
		

		if( listener.waitForTransform("/map", "/odom", now, ros::Duration(1.0)) ) 
			listener.lookupTransform("/map", "/odom",  now, transform);
			
		tf::Vector3 error_vector(dist(generator),dist(generator),0);
		
		error_transform.setRotation(q);
		error_transform.setOrigin(error_vector);
		
		transform.operator*=(error_transform);
		transform.getRotation().normalize();

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom_error"));
		rate.sleep();
		}
		else{
			if( listener.waitForTransform("/map", "/odom", now, ros::Duration(1.0)) ) 
			listener.lookupTransform("/map", "/odom",  now, transform);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom_error"));
			rate.sleep();
		}
	
	
	}
	
	}

