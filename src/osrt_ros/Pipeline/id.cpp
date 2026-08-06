#include "osrt_ros/Pipeline/id_common.h"
#include "ros/duration.h"
#include "ros/exception.h"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <SimTKcommon/internal/Vec.h>
#include <SimTKcommon/internal/Vector_.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"

#include "osrt_ros/Pipeline/id.h"
#include "opensimrt_bridge/conversions/message_convs.h"
#include "tf2/convert.h"


using namespace std;
using namespace OpenSim;
;
using namespace OpenSimRT;

Pipeline::Id::Id(): 
	tfListener(tfBuffer),
	sync_real_wrenches_exact(grf_exact_wrench_policy(50),sub,sub_wl,sub_wr), 
	sync_real_wrenches_aprox(grf_approx_wrench_policy(50),sub,sub_wl,sub_wr), 
	//sync_filtered_real_wrenches(sub_filtered,sub_wl,sub_wr,10),
	sync_filtered_real_wrenches_aprox(grf_approx_wrench_filtered_policy(50),sub_filtered,sub_wl,sub_wr),
	sync_filtered_real_wrenches_exact(grf_exact_wrench_filtered_policy(50),sub_filtered,sub_wl,sub_wr)
{
	ROS_INFO_STREAM("called Id constructor.");
	//cout << "this is being called out of order." << endl;

	nh.param<std::string>("left_foot_tf_name", left_foot_tf_name, "left_foot_forceplate");
	nh.param<std::string>("right_foot_tf_name", right_foot_tf_name, "right_foot_forceplate");
	nh.param<std::string>("grf_reference_frame", grf_reference_frame, "map");
	nh.param<bool>("use_exact_sync", use_exact_sync, true);




}

Pipeline::Id::~Id()
{
	ROS_INFO_STREAM("Shutting down Id");
}


void Pipeline::Id::onInit() {
	ROS_DEBUG_STREAM("called Id onInit");
	Pipeline::IdCommon::onInit();

	//the message filters for wrenches	
	sub_wl.subscribe(nh,"left_wrench",100);
	sub_wr.subscribe(nh,"right_wrench",100);
	if (use_exact_sync)
	{
		ROS_WARN_STREAM("Using exact time sync.");
		sync_real_wrenches_exact.connectInput(sub,sub_wl,sub_wr);
		sync_real_wrenches_exact.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches,this, _1,_2,_3));
		sync_filtered_real_wrenches_exact.connectInput(sub_filtered,sub_wl,sub_wr);
		sync_filtered_real_wrenches_exact.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches_filtered,this, _1,_2,_3));
	}
	else
	{
		ROS_WARN_STREAM("Using ApproximateTime sync.");
		sync_real_wrenches_aprox.connectInput(sub,sub_wl,sub_wr);
		sync_real_wrenches_aprox.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches,this, _1,_2,_3));
		sync_filtered_real_wrenches_aprox.connectInput(sub_filtered,sub_wl,sub_wr);
		sync_filtered_real_wrenches_aprox.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches_filtered,this, _1,_2,_3));



	}

	//CRAZY DEBUG

	pub_grf_left = nh.advertise<opensimrt_msgs::CommonTimed>("debug_grf_left", 1000);
	pub_grf_right = nh.advertise<opensimrt_msgs::CommonTimed>("debug_grf_right", 1000);
	pub_ik = nh.advertise<opensimrt_msgs::CommonTimed>("debug_ik", 1000);
	pub_cop_left = nh.advertise<opensimrt_msgs::CommonTimed>("debug_cop_left", 1000);
	pub_cop_right = nh.advertise<opensimrt_msgs::CommonTimed>("debug_cop_right", 1000);

}



//oh, these exist in Osb already...
opensimrt_msgs::CommonTimed Pipeline::Id::conv_ik_to_msg(std_msgs::Header h, SimTK::Vector ik)
{
	opensimrt_msgs::CommonTimed msg;	
	msg.header =h;
	for (auto q:ik)
		msg.data.push_back(q);
	return msg;

}
opensimrt_msgs::CommonTimed Pipeline::Id::conv_grf_to_msg(std_msgs::Header h, ExternalWrench::Input ow)
{
	opensimrt_msgs::CommonTimed msg;	
	msg.header = h;
	msg.data.push_back(ow.point[0]);
	msg.data.push_back(ow.point[1]);
	msg.data.push_back(ow.point[2]);
	msg.data.push_back(ow.force[0]);
	msg.data.push_back(ow.force[1]);
	msg.data.push_back(ow.force[2]);
	msg.data.push_back(ow.torque[0]);
	msg.data.push_back(ow.torque[1]);
	msg.data.push_back(ow.torque[2]);



	return msg;
}


bool Pipeline::Id::parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frame, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame, ExternalWrench::Input* wO )
{
	ROS_INFO_STREAM("times.. wrench header:"<< w->header << "\nnow: "<<ros::Time::now());
	auto sometime = w->header.stamp; //I hate myself.
	//auto sometime = ros::Time(0); //I hate myself.
	
	//TODO:I actually should read the ref_frame from the wrench like a normal person.
	//NO. we are using this for both the wrenches, so it will use the value for the wrong side if the transform fails!. 
	// now get the translations from the transform
	// for the untranslated version I just want to get the reference in their own coordinate reference frame
	geometry_msgs::TransformStamped nulltransform, actualtransform,inv_t;
	try
	{
		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);
		if (false)
		{
			size_t found = ref_frame.find("left");
			if (found!=std::string::npos)
			{//left
			 //i gotta do 2 lookups. one to the frigging food
			 // "left_filtered " -"calcn_l"
			 //	"calcn_l" - "subject_opensim"
				auto foot_cop = tfBuffer.lookupTransform(ref_frame, "calcn_l", sometime);
				auto foot_pos = tfBuffer.lookupTransform("calcn_l", grf_reference_frame, sometime);
				opensimrt_msgs::CommonTimed msg_cop;
				msg_cop.header.stamp = ros::Time::now();
				msg_cop.data.push_back( foot_cop.transform.translation.x);
				msg_cop.data.push_back( foot_cop.transform.translation.y);
				msg_cop.data.push_back( foot_cop.transform.translation.z);
				msg_cop.data.push_back(foot_pos.transform.translation.x);
				msg_cop.data.push_back(foot_pos.transform.translation.y);
				msg_cop.data.push_back(foot_pos.transform.translation.z);
				pub_cop_left.publish(msg_cop);
			}
			else
			{//right
			 //"right_filtered" - "calcn_r"
			 //	"calcn_r" - "subject_opensim"
				auto foot_cop = tfBuffer.lookupTransform(ref_frame, "calcn_r", sometime);
				auto foot_pos = tfBuffer.lookupTransform("calcn_r", grf_reference_frame, sometime);
				opensimrt_msgs::CommonTimed msg_cop;
				msg_cop.header.stamp = ros::Time::now();
				msg_cop.data.push_back( foot_cop.transform.translation.x);
				msg_cop.data.push_back( foot_cop.transform.translation.y);
				msg_cop.data.push_back( foot_cop.transform.translation.z);
				msg_cop.data.push_back(foot_pos.transform.translation.x);
				msg_cop.data.push_back(foot_pos.transform.translation.y);
				msg_cop.data.push_back(foot_pos.transform.translation.z);
				pub_cop_right.publish(msg_cop);

			}

		}

		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);

		nulltransform = tfBuffer.lookupTransform(grf_reference_frame, ref_frame, sometime);
		//nulltransform = tfBuffer.lookupTransform("subject_opensim", ref_frame, ros::Time(0));
		wO->point[0] = nulltransform.transform.translation.x;
		wO->point[1] = nulltransform.transform.translation.y;
		wO->point[2] = nulltransform.transform.translation.z;
		//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
		//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
		//ROS_DEBUG_STREAM("null transform::\n" << nulltransform);
		//ROS_DEBUG_STREAM("actual transform" << actualtransform);
		//ROS_DEBUG_STREAM("inverse transform" << inv_t);
		//inv_t converts back to opensim
		
		//now convert it:
		tf2::Quaternion q ;
		tf2::fromMsg(nulltransform.transform.rotation,q);
		tf2::Vector3 v_force_orig, v_torque_orig;
		tf2::fromMsg(w->wrench.force, v_force_orig);
		tf2::fromMsg(w->wrench.torque, v_torque_orig);

		tf2::Vector3 v_force_new = quatRotate(q, v_force_orig);
		wO->force[0] =v_force_new.x();
		wO->force[1] =v_force_new.y();
		wO->force[2] =v_force_new.z();
		tf2::Vector3 v_torque_new = quatRotate(q, v_torque_orig);
		wO->torque[0] =v_torque_new.x();
		wO->torque[1] =v_torque_new.y();
		wO->torque[2] =v_torque_new.z();

	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("tutu-loo: message_convs.cpp parse_message transform exception: %s",ex.what());
		//ros::Duration(1.0).sleep();
		return false;
	}

	//ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
	return true;

}
std::vector<OpenSimRT::ExternalWrench::Input> Pipeline::Id::get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr, std::string right_foot_tf_name, std::string left_foot_tf_name, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame)

{

	// TODO: get wrench message!!!!!!!!!!
	std::vector<ExternalWrench::Input> wrenches;
	OpenSimRT::ExternalWrench::Input nullWrench;
	nullWrench.force = SimTK::Vec3{0,0,0};
	nullWrench.torque = SimTK::Vec3{0,0,0};
	nullWrench.point = SimTK::Vec3{0,0,0};
	
	static OpenSimRT::ExternalWrench::Input grfRightWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gRw = new OpenSimRT::ExternalWrench::Input; 
	if(parse_message(wr, right_foot_tf_name, tfBuffer, grf_reference_frame, gRw))
		grfRightWrench = *gRw;
	//cout << "left wrench.";
	ROS_DEBUG_STREAM("rw");
	static OpenSimRT::ExternalWrench::Input grfLeftWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gLw = new OpenSimRT::ExternalWrench::Input; 
	if(parse_message(wl, left_foot_tf_name, tfBuffer, grf_reference_frame, gLw))
		grfLeftWrench = *gLw;
	ROS_DEBUG_STREAM("lw");
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;


}


void Pipeline::Id::callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik & wrenches",message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback_real_wrenches"); 
	double filtered_t;
	addEvent("id: getting iks", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	addEvent("id: getting real wrenches", newEvents1);
	auto grfs = Pipeline::Id::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	addEvent("calling run function of id", newEvents1);

	run(message_ik->header, filtered_t, iks, grfs, newEvents1);	
}

void Pipeline::Id::callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik filtered & wrenches", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	//cant find the right copy constructor syntax. will for loop it
	addEvent("id: getting iks already filtered.", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik);
	addEvent("id: getting real wrenches", newEvents1);
	auto grfs = Pipeline::Id::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	addEvent("calling run function of id", newEvents1);
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}


