// Server side implementation of a ROS TF server model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "osrt_ros/UIMU/TfServer.h"

#include <iostream>
#include <sstream>
#include <iterator>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>


// Driver code
TfServer::TfServer (std::vector<std::string> tf_names, std::string tf_frame_prefix) {

	ROS_INFO("Using tf server");
	///read one tf and make it for everyone
	tf::TransformListener listener;
	set_tfs(tf_names, tf_frame_prefix);

}

TfServer::~TfServer(void)
{
}

void TfServer::set_tfs(std::vector<std::string> tf_names, std::string tf_frame_prefix)
{

	//tf_strs = tf_names;
	if (tf_names.size() == 0)
		ROS_FATAL("TFServer: NO TF NAMES");
	for (auto i:tf_names)
	{
		std::string used_frame = tf_frame_prefix+"/" + i;
		ROS_DEBUG_STREAM("TfServer: Using reference frame: " << used_frame);
		tf_strs.push_back(used_frame);
	}
}

void TfServer::set_world_reference(std::string world_name)
{
	world_tf_reference = world_name;
	ROS_INFO("Setting world_tf_reference to %s", world_tf_reference.c_str());	
}

//std::vector<double> TfServer::receive()
bool TfServer::receive()
{
	ROS_DEBUG_STREAM("TFServer receive called: ");
	std::vector<double> combined_imu_data_vec;
	// now i need to set this combined_imu_data_vec with the values i read for the quaternions somehow
	//std::vector<std::string> tf_strs = {"/a", "b", "c"};	
	double time = ros::Time::now().toSec();// i need to get this from the transform somehow
	combined_imu_data_vec.push_back(time);

	for (auto i:tf_strs)
	{
		readTransform(i); //updates map
		std::vector<double> anImu = readTransformIntoOpensim(i);
		combined_imu_data_vec.insert(combined_imu_data_vec.end(),anImu.begin(), anImu.end());
	}
	//for( auto i:combined_imu_data_vec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	ROS_DEBUG_STREAM("TFServer combined imu data vec size after reading: " << combined_imu_data_vec.size());

	output = combined_imu_data_vec;
	return true;	

}

void TfServer::readTransform(std::string tf_name)
{
	ROS_DEBUG_STREAM("TFServer: Trying to find transform " << tf_name);
	tf::StampedTransform transform;
	try{
		//listener.waitForTransform(tf_name, world_tf_reference, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform(world_tf_reference, tf_name, ros::Time(0), transform); // this would give OpenSim the raw quaternions, however, it seems to be incorrect as it is not possible to find the correct R matrix to put all the axis in the correct orientation. To solve this we either changed the order of some vectors or inverted w.
		//listener.lookupTransform(tf_name, world_tf_reference, ros::Time(0), transform); //flipped, new attempt to try to avoid -w
	}
	catch (tf::TransformException& ex){
		ROS_ERROR_THROTTLE(5,"TFServer: Orientation: Transform exception! %s",ex.what());
	}
	last_transforms[tf_name] = transform;
}
std::vector<double> TfServer::readTransformIntoOpensim(std::string tf_name)
{
	auto transform = last_transforms[tf_name];
	std::vector<double> imu_data_vec;
	// now i need to set this imu_data_vec with the values i read for the quaternions somehow

	auto imu_q = transform.getRotation();


	ROS_DEBUG_STREAM("TFServer: tf_name: " << tf_name << " transform:\n " 	
			<< "w: " << imu_q.getW() 
			//<< " " << imu_q.w() 
			<< "\n" 
			<< "x: " << imu_q.getX() 
			//<< " " << imu_q.x() 
			<< "\n" 
			<< "y: " << imu_q.getY() 
			//<< " " << imu_q.y() 
			<< "\n"
			<< "z: " << imu_q.getZ() 
			//<< " " << imu_q.z() 
			<< "\n");
	//this is converting from ROS quaternions to OPENSIM quaternions. Is this correct?

	imu_data_vec.push_back(imu_q.w());
	imu_data_vec.push_back(imu_q.x());
	imu_data_vec.push_back(imu_q.y());
	imu_data_vec.push_back(imu_q.z());
	// now it is a bunch of zeros
	// TODO: maybe read other data and place here? this will be a lot of rather useless work
	imu_data_vec.insert(imu_data_vec.end(), 14, -1.010 );
	//for( auto i:imu_data_vec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	//ROS_INFO_STREAM("THIS THING:" << imu_data_vec.size());

	return imu_data_vec;

}

