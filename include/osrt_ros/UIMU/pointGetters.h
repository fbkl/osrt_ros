#include "XmlRpcException.h"
#include "XmlRpcValue.h"
#include "geometry_msgs/TransformStamped.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/UIMU/IMUCalibrator.h"
//#include "INIReader.h"
#include "InverseKinematics.h"
#include "SignalProcessing.h"
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
//#include <OpenSim/Common/STOFileAdapter.h>

#include "osrt_ros/parameters.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/PoseStamped.h"
#include "opensimrt_msgs/Labels.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Array.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/ReinitOnCopy.h>
#include <chrono>
#include <exception>
#include <string>
#include <unordered_map>
#include <vector>
#include "osrt_ros/UIMU/TfServer.h"
#include "Ros/include/common_node.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

#include <dynamic_reconfigure/server.h>
#include <osrt_ros/UIMUConfig.h>
#include <osrt_ros/headingConfig.h>
#include "osrt_ros/events.h"
#include "opensimrt_bridge/conversions/message_convs.h"


#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>

#include <cmath> // Required for NAN. also cfloat is the c++26 version

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

const std::string bar("\n======================================================\n");

#define ROS_YE(x) ROS_INFO_STREAM( yellow << x << reset)

typedef std::pair<SimTK::Array_<SimTK::Vec3>,SimTK::Array_< SimTK::Real>> TransObs; 

class MyMarker
{
	public:
		std::string this_marker_name;
		double x;
		double y;
		double z;
		std::string this_marker_tf;



};


class GetPoint
{
	public:
		std::vector<std::string> markerNames;
		ros::NodeHandle nh{"~/marker"};
		std::unordered_map<std::string, std::string> markerDefList;
		XmlRpc::XmlRpcValue markerList;
		std::vector<MyMarker> mmList;
		double last_time{0.0};

		GetPoint() 
		{

			try{	
				nh.getParam("observation_order", markerList);
				if(markerList.valid())
					ROS_WARN_STREAM("AR markerObservationOrder:" << markerList.toXml());
				else 
					throw(XmlRpc::XmlRpcException("Couldn't parse observation_order"));
				//ROS_ASSERT(markerList.getType() == XmlRpc::XmlRpcValue::TypeArray); //

				if (markerList.size() == 0)
				{
					ROS_FATAL("AR Marker observation order not defined!");
					throw(std::invalid_argument("markerNames not defined."));
				}
				else
				{
					for (int32_t i = 0; i < markerList.size(); ++i) 
					{
						ROS_INFO("AR getpoint: Entered loop");
						MyMarker a;

						XmlRpc::XmlRpcValue markerDef = markerList[i]; 
						ROS_INFO("AR getpoint: Loaded MarkerDef");
						a.this_marker_name = (std::string)markerDef["marker_name"];
						a.this_marker_tf = (std::string)markerDef["marker_tf"];
						ROS_INFO_STREAM("AR getpoint: Assigned name: " << a.this_marker_name);
						markerNames.push_back(a.this_marker_name);
						ROS_INFO_STREAM("AR getpoint: ADDED TO MARKER LIST: " << magenta << markerNames.back() );
						XmlRpc::XmlRpcValue this_marker_default_position = markerDef["default_position"];
						ROS_INFO_STREAM(cyan <<"getpoitn read default_position ok"<<reset);
						ROS_ASSERT(this_marker_default_position.getType() == XmlRpc::XmlRpcValue::TypeArray); //
						a.x = this_marker_default_position[0];
						a.y = this_marker_default_position[1];
						a.z = this_marker_default_position[2];
						ROS_INFO_STREAM(cyan <<"getpoint setting x,y,z okay"<<reset);

						mmList.push_back(a);
					}
				}
			}
			catch(XmlRpc::XmlRpcException& e)
			{
				ROS_ERROR_STREAM("AR getpoint: Could not setup markers" << e.getMessage());
			}
			ROS_INFO("AR getpoint: Finished setting up markers");

		}

		virtual TransObs get_translations()
		{
			TransObs markerObservations;

			//remove and place in the derived

			ROS_ERROR("abstract implementation shouldn't be used, will return empty observations!!!");
			return markerObservations;
		}
};
class GetPointFromSomeTF: public GetPoint // to make this a threaded implementation we need also do run a rated loop here, not going to do that yet, but if you want fast AR, you probably should try it.
{
	//tf::TransformListener tl;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	std::string tf_frame_prefix;
	std::string world_tf_reference;
	double tf_timeout;
	std::vector<std::string> tfNames;
	std::unordered_map<std::string, geometry_msgs::TransformStamped> latest_marker_tfs;
	public:
	GetPointFromSomeTF(): tfListener(tfBuffer), tf_timeout(0.0) 
	{
		nh.param<double>("tf_timeout",tf_timeout,0.01); // here the timeout was too large. we should probably warn about it
		nh.param<std::string>("world_tf_reference",world_tf_reference,"map");
		nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
		try{	
			XmlRpc::XmlRpcValue markerList;
			nh.getParam("observation_order", markerList);
			if(markerList.valid())
				ROS_YE("AR markerObservationOrder:" << markerList.toXml());
			else 
				throw(XmlRpc::XmlRpcException("Couldn't parse observation_order"));
			//ROS_ASSERT(markerList.getType() == XmlRpc::XmlRpcValue::TypeArray); //

			ROS_INFO_STREAM("AR: parsing points and tf map");
			ROS_INFO_STREAM("AR: Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
			for (int32_t i = 0; i < markerList.size(); ++i) 
			{
				ROS_INFO("AR: Entered loop");

				auto a = mmList[i];

				std::string this_marker_tf   = tf_frame_prefix + "/" + a.this_marker_tf;
				ROS_INFO_STREAM("AR: assigned tf: " << this_marker_tf);
				markerDefList[a.this_marker_name] = this_marker_tf;
				ROS_INFO("AR: Assigned to map");
				tfNames.push_back(this_marker_tf);
				ROS_INFO("AR: ADDED TO TF LIST");
				//We also add the default position of the marker so that if the AR is broken, it doesnt affect the measurement too much
				geometry_msgs::TransformStamped transform;
				transform.transform.translation.x = a.x;
				transform.transform.translation.y = a.y;
				transform.transform.translation.z = a.z;
				latest_marker_tfs[this_marker_tf] = transform;


				ROS_INFO_STREAM(cyan <<"FINISHED SETTING UP ONE TF MARKER AT LEAST"<<reset);
				//ROS_ASSERT(markerDef[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			}
			//for (auto& marker:markerNames)
			//	marker+=tf_frame_prefix;
		}
		catch(XmlRpc::XmlRpcException& e)
		{
			ROS_ERROR_STREAM("AR: Could not setup markers" << e.getMessage());
		}
		ROS_INFO("AR: Finished setting up markers");

	}
	TransObs get_translations() override
	{
		TransObs markerObservations;

		SimTK::Array_<SimTK::Vec3> actualObservations;
		SimTK::Array_<SimTK::Real> accuracyOfObservations;
		for (const auto& [this_marker_name, this_marker_tf] : markerDefList)
		{
			SimTK::Vec3 v;
			try{
				geometry_msgs::TransformStamped transform;
				//transform = tfBuffer.lookupTransform( this_marker_tf, world_tf_reference, ros::Time(0), ros::Duration(tf_timeout) ); //
				transform = tfBuffer.lookupTransform( world_tf_reference, this_marker_tf, ros::Time(0), ros::Duration(tf_timeout) ); //
				//transform = tfBuffer.lookupTransform( "map", this_marker_tf, ros::Time(0), ros::Duration(tf_timeout) ); //
				latest_marker_tfs[this_marker_tf] = transform;
				last_time = transform.header.stamp.toSec();
			}
			catch (tf::TransformException& ex){
				ROS_ERROR_THROTTLE(60,"AR: Translation part Transform exception! %s",ex.what());
			}
			auto transform = latest_marker_tfs[this_marker_tf];
			//if you are in a hurry just hard code the transform here because we just want it to work now.
			v.set(0, transform.transform.translation.x);
			v.set(1, transform.transform.translation.y);
			v.set(2, transform.transform.translation.z);
			actualObservations.push_back(v);
			accuracyOfObservations.push_back( 1); // we don't have a marker quality value here

		}
		markerObservations.first = actualObservations;
		markerObservations.second = accuracyOfObservations;

		return markerObservations;
	}
};

class GetPointFromMarkers:public GetPoint
{
 		std::shared_ptr<std::mutex> mtx_;
	ros::Subscriber marker_sub;
	std::unordered_map<std::string, vicon_bridge::Marker> marker_map;
	
	TransObs markerObservations;

	chrono::high_resolution_clock::time_point t0 ;
	double multiplier=0.001;
	void callback(const vicon_bridge::MarkersPtr& msg)
	{
		chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;
		std::lock_guard<std::mutex> lock(*mtx_);
		//not in the right order, we need a freaking map, right?	
		for (const auto& marker:msg->markers)
		{
			marker_map[marker.marker_name] = marker;
		}
		last_time = msg->header.stamp.toSec();
		
		SimTK::Array_<SimTK::Vec3> actualObservations;
		SimTK::Array_<SimTK::Real> accuracyOfObservations;

		//TODO:REPLACE
		for (const auto& this_marker_name:markerNames)
			//			for (const auto& [this_marker_name, this_Marker] : marker_map)
		{
			SimTK::Vec3 v;

			// now we find the latest marker in the unordered map
			const auto& this_Marker = marker_map[this_marker_name];

			//if you are in a hurry just hard code the transform here because we just want it to work now.
			v.set(0, this_Marker.translation.x*multiplier);
			v.set(1, this_Marker.translation.y*multiplier);
			v.set(2, this_Marker.translation.z*multiplier);
			// since the name order is fixed, this order should also be fixed, so it is okay
			
			actualObservations.push_back(v);
			accuracyOfObservations.push_back(this_Marker.occluded ? NAN : 1.0); // according to the docs from SimTK, this is what we want

		}
		markerObservations.first = actualObservations;
		markerObservations.second = accuracyOfObservations;
		
		chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;
		double dur_own_code = chrono::duration_cast<chrono::nanoseconds>(t2-t1).count();
		double dur_between = chrono::duration_cast<chrono::nanoseconds>(t2-t0).count();
		ROS_YE(bar << "time between calls duration in ns:"<<magenta<<dur_between<<"\nduration own call"<<dur_own_code<<bar<<"fps:"<<1000000000.0/dur_between<<"fps own:"<<1000000000.0/dur_own_code<<bar <<reset);

			t0 = t2;

	}
	public:

	GetPointFromMarkers() : mtx_(std::make_shared<std::mutex>())

	{
		t0=chrono::high_resolution_clock::now() ;
		std::lock_guard<std::mutex> lock(*mtx_);

		try{	
			ROS_INFO_STREAM("AR: parsing points and vicon marker map");
			for (int32_t i = 0; i < markerList.size(); ++i)  //what is this doing? shouldnt it be only the thing below?
			{
				ROS_INFO("AR vicon : Entered loop");
				auto a = mmList[i];
				vicon_bridge::Marker this_marker;		

				this_marker.translation.x = a.x*multiplier;
				this_marker.translation.y = a.y*multiplier;
				this_marker.translation.z = a.z*multiplier;
				this_marker.marker_name = a.this_marker_name;
				//latest_marker_vec.push_back(this_marker); //NO! i am initializing the MAP here, not this vector which we dont use
				marker_map[a.this_marker_name] = this_marker;
				ROS_INFO_STREAM(cyan <<"FINISHED SETTING UP ONE VICON MARKER AT LEAST"<<reset);
			}
			//for (auto& marker:markerNames)
			//	marker+=tf_frame_prefix;
		}
		catch(XmlRpc::XmlRpcException& e)
		{
			ROS_ERROR_STREAM("AR: Could not setup vicon markers" << e.getMessage());
		}
		ROS_INFO("AR: Finished setting up vicon markers");
		
		SimTK::Array_<SimTK::Vec3> actualObservations;
		SimTK::Array_<SimTK::Real> accuracyOfObservations;
		for (const auto& this_marker_name:markerNames)
			//			for (const auto& [this_marker_name, this_Marker] : marker_map)
		{
			SimTK::Vec3 v;

			// now we find the latest marker in the unordered map
			const auto& this_Marker = marker_map[this_marker_name];

			//if you are in a hurry just hard code the transform here because we just want it to work now.
			v.set(0, this_Marker.translation.x*multiplier);
			v.set(1, this_Marker.translation.y*multiplier);
			v.set(2, this_Marker.translation.z*multiplier);
			// since the name order is fixed, this order should also be fixed, so it is okay
			
			actualObservations.push_back(v);
			accuracyOfObservations.push_back(this_Marker.occluded ? NAN : 1.0); // according to the docs from SimTK, this is what we want

		}
		markerObservations.first = actualObservations;
		markerObservations.second = accuracyOfObservations;

		marker_sub = nh.subscribe("/vicon/markers", 1,&GetPointFromMarkers::callback, this,ros::TransportHints().tcpNoDelay());
	}
	TransObs get_translations() override
	{
			std::lock_guard<std::mutex> lock(*mtx_);
		
		return markerObservations;
	}
};




