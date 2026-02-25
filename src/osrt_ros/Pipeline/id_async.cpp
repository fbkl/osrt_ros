#include "osrt_ros/Pipeline/id_async.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/GroundProjectionOrientationAtTimeSrv.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "tf/LinearMath/Vector3.h"
#include "tf2/LinearMath/Vector3.h"
#include <exception>
#include <memory>
#include <stdexcept>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string blue("\033[0;34m");
const std::string reset("\033[0m");

Pipeline::WrenchSubscriber::WrenchSubscriber(std::string wrench_name_prefix_, std::string calcn_frame_):
	tfListener(tfBuffer), calcn_frame(calcn_frame_), wrench_name_prefix(wrench_name_prefix_)
{
	ROS_DEBUG_STREAM("called WrenchSubscriber constructor with params.");
}

void Pipeline::WrenchSubscriber::onInit()
{
	ROS_DEBUG_STREAM("called WrenchSubscriber onInit");
	nh.param<bool>("use_grfm_filter", use_grfm_filter, false);
	nh.param<int>("max_buffer_length", max_buffer_length, 50);
	sub = nh.subscribe(wrench_name_prefix+"/wrench",10, &WrenchSubscriber::callback, this);

	pub_grf = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_grf", 10);
	pub_cop = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_cop", 10);
	nh.param<std::string>(wrench_name_prefix+"_foot_tf_name", foot_tf_name, wrench_name_prefix+"_foot_forceplate");
	nh.param<std::string>(wrench_name_prefix+"_reference_frame", grf_reference_frame, "map");
	nh.param<bool>(wrench_name_prefix+"_no_rotation", no_rotation, false);
	nh.param<bool>(wrench_name_prefix+"_get_external_orientation", get_external_orientation, false);

	if(no_rotation)
		ROS_WARN_STREAM("no_rotation to be applied to: "<< wrench_name_prefix);
	else
		ROS_INFO_STREAM("Applying rotation to" << wrench_name_prefix);

	if(get_external_orientation)
	{
		ROS_INFO_STREAM("Using External Orientation provider");
		if(no_rotation)
			ROS_WARN_STREAM("External Orientation is set and it overrides no_rotation option! Using external orientation service!");
		ground_orientation_client = nh.serviceClient<opensimrt_msgs::GroundProjectionOrientationAtTimeSrv>("get_ground_projection_orientation");

	}

	//throw (std::runtime_error("bye"));
}

void Pipeline::WrenchSubscriber::callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
	//ROS_DEBUG_STREAM("reached a wrench subscriber!");
	//places the wrench in the buffer
	geometry_msgs::WrenchStamped my_wrench = *msg.get();
	wrenchBuffer.push_back(my_wrench);
	ROS_DEBUG_STREAM(wrench_name_prefix+": what i think i am saving" << my_wrench);
	//ROS_INFO_STREAM(wrench_name_prefix+": wrench time:" << my_wrench.header.stamp);
	ROS_DEBUG_STREAM(wrench_name_prefix+": buffer size" <<wrenchBuffer.size());
	while (wrenchBuffer.size()>max_buffer_length)
	{
		wrenchBuffer.pop_front();
	}

}
const geometry_msgs::WrenchStamped Pipeline::WrenchSubscriber::find_wrench_in_buffer(const std_msgs::Header::_stamp_type timestamp)
{
	auto best_wrench = geometry_msgs::WrenchStamped();
	double old_best_wrench_time_offset = 3000;
	double this_wrench_time = 3000;
	static size_t too_fast_counter = 0;
	static size_t too_slow_counter = 0;
	if (!wrenchBuffer.empty())
	{
		//int i = -1;
		for (auto &w:wrenchBuffer)
		{
			this_wrench_time = fabs((w.header.stamp - timestamp).toSec());
			if (this_wrench_time<old_best_wrench_time_offset)
			{
				old_best_wrench_time_offset = this_wrench_time;
				//ROS_DEBUG_STREAM("I found a better wrench:\n" << w) ;
				//ROS_DEBUG_STREAM("The wrench just before it was: "<< best_wrench.header.stamp );
				best_wrench = w;
				
				//i++;
			}
		}
		//ROS_INFO_STREAM("\nwrench i got has the timestamp: " << best_wrench.header.stamp << "\ndesired stamp was:" << timestamp <<"\ntime difference: "<< best_wrench.header.stamp - timestamp);
		//ROS_INFO_STREAM("wrench I got was in position: [" << i << "/" << max_buffer_length << "] at " << 100.0*float(i)/max_buffer_length << "%");
		//ROS_INFO_STREAM("first time in buffer	:" <<wrenchBuffer.front().header.stamp);
		//ROS_INFO_STREAM("last time in buffer	:" <<wrenchBuffer.back().header.stamp);
		//ROS_INFO_STREAM("desired time 		:" <<timestamp);
		bool ik_too_slow = wrenchBuffer.front().header.stamp >timestamp;
		bool ik_too_fast = wrenchBuffer.back().header.stamp < timestamp;
			
		static bool keep_showing_errors = true;
		if (ik_too_fast)
		{
				ROS_FATAL_STREAM_ONCE("Could not find a wrench that matched the desired timestamp.\n\t" << blue <<"IK is too fast!\n\tcounts: "<< too_fast_counter << reset);
				too_fast_counter++;
		}

		if (ik_too_slow)
		{	
				ROS_FATAL_STREAM_ONCE("Could not find a wrench that matched the desired timestamp.\n\t"<< green <<"IK too slow!\n\tcounts: " << too_slow_counter <<reset);
		}
		if ((ik_too_fast || ik_too_slow)&& keep_showing_errors)
		{
			if ((timestamp-wrenchBuffer.back().header.stamp).toSec() > 4)
			{
				ROS_WARN_STREAM_ONCE("IK delay is larger than " << 4 << "seconds. Did the trial end?"); // the condition here for ending is that i am not receiving any more wrenches
				//keep_showing_errors = false;
			}

			else
			{	ROS_INFO_STREAM_THROTTLE(2,	"\nfirst time in buffer	:" <<wrenchBuffer.front().header.stamp <<
							"\nlast time in buffer	:" <<wrenchBuffer.back().header.stamp <<
							"\ndesired time		:" <<timestamp);
			}
		}
	}
	else
	{ //This should never happen
		ROS_FATAL_STREAM("Wrench Buffer is empty!");
		too_fast_counter++;
	}
	//ROS_INFO_STREAM(best_wrench.wrench.force);
	return best_wrench;
}

double magnitude(tf2::Vector3 v)
{
	double x =v.getX(); 
	double y =v.getY(); 
	double z =v.getZ(); 
	double mag = sqrt(x*x+y*y+z*z);
	return mag;
}

void show_diff(std::string what,tf2::Vector3 v0, tf2::Vector3 v1)
{
	static int count = 0;
	static double max_diff = 0;
	static double diff_sum = 0;

	static double relative_avg_error_sum = 0;

		double init_mag = magnitude(v0); 

		double end_mag = magnitude(v1); 

		double diff = init_mag-end_mag;
		
		diff_sum+=diff;

		double relative_error = diff/init_mag;

		relative_avg_error_sum+=relative_error;

		ROS_INFO_STREAM("relative_error percentage :" << relative_error * 100 << "% " );
		ROS_INFO_STREAM("initial " << what <<" mag: " << init_mag << " mag after rotation: "<< end_mag <<" diff:" << diff );

	if (max_diff < diff)
	{
		max_diff = diff;
	}

		ROS_WARN_STREAM("max diff: " << max_diff);

		ROS_WARN_STREAM("avg_diff: " << diff_sum/count );

		ROS_WARN_STREAM("relative_error " << relative_avg_error_sum /count);

		count++;
}



bool Pipeline::WrenchSubscriber::get_wrench(const std_msgs::Header::_stamp_type timestamp, ExternalWrench::Input* wO )
{
	//find the actual wrench I need based on timestamp
	auto w = find_wrench_in_buffer(timestamp);
	auto now = ros::Time::now();
	if (w.header.frame_id == "")
	{
		ROS_ERROR("Header frame_id is empty!!!");
		return false;
	}
	//ROS_INFO_STREAM("timing information:\nIK stamp: "<< timestamp <<"\nfound wrench header stamp:"<< w.header.stamp << "\nnow: "<< now <<"\nDelay of this node:" << now-timestamp );
	//auto sometime = w.header.stamp; //I hate myself.
	//auto sometime = ros::Time(0); //I hate myself.

	auto sometime= timestamp;
	//TODO:I actually should read the ref_frame from the wrench like a normal person.
	//NO. we are using this for both the wrenches, so it will use the value for the wrong side if the transform fails!. 
	// now get the translations from the transform
	// for the untranslated version I just want to get the reference in their own coordinate reference frame
	geometry_msgs::TransformStamped nulltransform, actualtransform,inv_t;
	nulltransform.transform.rotation.w = 1;
	//ROS_DEBUG_STREAM("nulltransform" << nulltransform.transform);
	try
	{
		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);
		if (false)
		{
			//i gotta do 2 lookups. one to the frigging food
			// "left_filtered " -"calcn_l"
			//	"calcn_l" - "subject_opensim"
			auto foot_cop = tfBuffer.lookupTransform(foot_tf_name, calcn_frame, sometime);
			auto foot_pos = tfBuffer.lookupTransform(calcn_frame, grf_reference_frame, sometime);
			opensimrt_msgs::CommonTimed msg_cop;
			msg_cop.header.stamp = ros::Time::now();
			msg_cop.data.push_back( foot_cop.transform.translation.x);
			msg_cop.data.push_back( foot_cop.transform.translation.y);
			msg_cop.data.push_back( foot_cop.transform.translation.z);
			msg_cop.data.push_back(foot_pos.transform.translation.x);
			msg_cop.data.push_back(foot_pos.transform.translation.y);
			msg_cop.data.push_back(foot_pos.transform.translation.z);
			pub_cop.publish(msg_cop);

		}
		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);

		actualtransform = tfBuffer.lookupTransform(grf_reference_frame, foot_tf_name, sometime);

		wO->point[0] = actualtransform.transform.translation.x;
		wO->point[1] = actualtransform.transform.translation.y;
		wO->point[2] = actualtransform.transform.translation.z;
		//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
		//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
		//ROS_DEBUG_STREAM("null transform::\n" << nulltransform);
		//ROS_DEBUG_STREAM("actual transform" << actualtransform);
		//ROS_DEBUG_STREAM("inverse transform" << inv_t);
		//inv_t converts back to opensim

		if (no_rotation)
			actualtransform.transform.rotation = nulltransform.transform.rotation;
		if (get_external_orientation)
		{
			ROS_INFO_STREAM("getting external orientation");
			opensimrt_msgs::GroundProjectionOrientationAtTimeSrv srv;
			srv.request.point_stamped.header = w.header; //will get the time of the found wrench.
			srv.request.point_stamped.point.x = actualtransform.transform.translation.x; 
			srv.request.point_stamped.point.y = actualtransform.transform.translation.y; 
			srv.request.point_stamped.point.z = actualtransform.transform.translation.z; 
			if (ground_orientation_client.call(srv)) {
				// Service call successful, process the response
				ROS_INFO("Received orientation: x=%f, y=%f, z=%f, w=%f", srv.response.orientation.x, srv.response.orientation.y, srv.response.orientation.z, srv.response.orientation.w);
			} else {
				// Service call failed
				ROS_ERROR("Failed to call service!!!");
				//return 1;
			}
			actualtransform.transform.rotation = srv.response.orientation; 
		}
		//now convert it:
		tf2::Quaternion q ;
		tf2::fromMsg(actualtransform.transform.rotation,q);
		tf2::Vector3 v_force_orig, v_torque_orig;
		tf2::fromMsg(w.wrench.force, v_force_orig);
		tf2::fromMsg(w.wrench.torque, v_torque_orig);

		tf2::Vector3 v_force_new = quatRotate(q, v_force_orig);
		wO->force[0] =v_force_new.x();
		wO->force[1] =v_force_new.y();
		wO->force[2] =v_force_new.z();

		//show_diff("force", v_force_orig,v_force_new);

		tf2::Vector3 v_torque_new = quatRotate(q, v_torque_orig);
		wO->torque[0] =v_torque_new.x();
		wO->torque[1] =v_torque_new.y();
		wO->torque[2] =v_torque_new.z();

	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("Could not find a transform: parse_message transform exception: %s",ex.what());
		//ROS_ERROR_ONCE("Could not find a transform: parse_message transform exception: %s",ex.what());
		//ros::Duration(1.0).sleep();
		return false;
	}
	//ROS_INFO_STREAM("I got some wrench. so far so good.");
	//ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
//	ROS_INFO_STREAM(wO->force[0]);
//	ROS_INFO_STREAM(wO->force[1]);
//	ROS_INFO_STREAM(wO->force[2]);
	//ROS_INFO_STREAM(wO->point[0]);
	//ROS_INFO_STREAM(wO->point[1]);
	//ROS_INFO_STREAM(wO->point[2]);
	return true;
}



std::vector<OpenSimRT::ExternalWrench::Input> Pipeline::IdAsync::get_wrench(const std_msgs::Header::_stamp_type timestamp)

{

	// TODO: get wrench message!!!!!!!!!!
	std::vector<ExternalWrench::Input> wrenches;
	OpenSimRT::ExternalWrench::Input nullWrench;
	nullWrench.force = Vec3{0,0,0};
	nullWrench.torque = Vec3{0,0,0};
	nullWrench.point = Vec3{0,0,0};

	//sanity check
	//
	//ROS_INFO_STREAM("this: \n" <<ros::Time::now()-timestamp<<"\nshould be the same delay as this:\n" <<ik_delay);
	// currently it works
	//ROS_INFO_STREAM("ik being evaluated now: "<< timestamp);

	static int num_sync_errors = 0;
	static OpenSimRT::ExternalWrench::Input grfRightWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gRw(&grfRightWrench); 
	if(wsR.get_wrench(timestamp, gRw))
		grfRightWrench = *gRw;
	else
	{
		ROS_DEBUG("did not find right wrench. using nullWrench! sync error count %d",num_sync_errors);
		ROS_WARN_ONCE("did not find right wrench. using nullWrench! trial will contain sync errors");
		num_sync_errors++;
	}
	//cout << "left wrench.";
	//ROS_DEBUG_STREAM("rw");
	static OpenSimRT::ExternalWrench::Input grfLeftWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gLw(&grfLeftWrench); 
	if(wsL.get_wrench(timestamp, gLw))
		grfLeftWrench = *gLw;
	else
	{
		ROS_WARN_ONCE("did not left right wrench. using nullWrench! trial will contain sync errors");
		ROS_DEBUG("did not find left wrench. using nullWrench! sync error count %d", num_sync_errors);
		num_sync_errors++;
	}
	//ROS_DEBUG_STREAM("lw");
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;


}

#define BUFFER_LENGTH 1000
#define READING_TIMEOUT 0.002

Pipeline::IdAsync::IdAsync(): 
	seq__(sub__, std::make_shared<ros::Duration>(0.3),ros::Duration(READING_TIMEOUT),BUFFER_LENGTH), 
	seq_filtered__(sub_filtered__, std::make_shared<ros::Duration>(0.3),ros::Duration(READING_TIMEOUT),BUFFER_LENGTH), 
	wsL("left", "calcn_l"), 
	wsR("right", "calcn_r")
{
	ROS_WARN_STREAM("Are you sure you dont want to set up a custom delay?");
	*ik_delay = ros::Duration(0.3);
}

Pipeline::IdAsync::IdAsync(std::shared_ptr<ros::Duration> ik_delay_): 
	ik_delay(ik_delay_),
	seq__(sub__, ik_delay, ros::Duration(READING_TIMEOUT),BUFFER_LENGTH), 
	seq_filtered__(sub_filtered__, ik_delay,ros::Duration(READING_TIMEOUT),BUFFER_LENGTH), 
	wsL("left", "calcn_l"), 
	wsR("right", "calcn_r")
{
	ROS_INFO_STREAM("subscribing with a delay of: "<<ik_delay->toSec()<<"s.");
}


Pipeline::IdAsync::~IdAsync()
{
	ROS_INFO_STREAM("Shutting down Id");
}

void Pipeline::IdAsync::reconfigure_delay_callback(osrt_ros::delayConfig &config, uint32_t level)
{
	ROS_INFO("attempt to set delay to:%f", config.global_delay);
	ROS_WARN("not implemented yet");

	ik_delay = std::make_shared<ros::Duration>(config.global_delay);
	seq__.modify_delay(ik_delay);
	seq_filtered__.modify_delay(ik_delay);
}

void Pipeline::IdAsync::onInit() {
	//TODO: this is technically wrong. if I am subscribing to the version with CommonTimed version, then I definetely want the second label as well, but it will fail if I am not subscribing to this, so this flag needs to be set only in that case
	//get_second_label = false;
	Pipeline::IdCommon::onInit();
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//TODO::::: HEre I want to subscribe with a timeSequencer!!!!!!!!!!!!
	//message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	//sub0.registerCallback(&Pipeline::IdAsync::callback0,this);
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////

	//this is also wrong because it will get destroyed after this is finished...
	//message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	//sub1.registerCallback(&Pipeline::IdAsync::callback1,this);

	//I will start all the wrenches now:
	wsL.onInit();
	wsR.onInit();


	pub_ik = nh.advertise<opensimrt_msgs::CommonTimed>("debug_ik", 10);
	//i need the labels and other stuff maybe
	//Ros::CommonNode::onInit();
	sub.unsubscribe();
	sub_filtered.unsubscribe();
	sub__.subscribe(nh, "input", 10);
	seq__.registerCallback(&Pipeline::IdAsync::callback0, this);	
	sub_filtered__.subscribe(nh, "input_filtered", 10);
	seq_filtered__.registerCallback(&Pipeline::IdAsync::callback_filtered, this);
	//I need a slow IK with a ton of delay.

}
ros::Time Pipeline::IdAsync::convert_time_stamp_to_the_past(const ros::Time timestamp0)
{
	ROS_FATAL_STREAM("not necessary, do not use!!!");
	auto t0secs = timestamp0.toSec();
	ROS_WARN_STREAM("this is the IK header time that is in the callback!!!!\n" << timestamp0);
	auto converted_time = ros::Time(t0secs - ik_delay->toSec());
	ROS_INFO_STREAM("converted_time" << converted_time);
	return converted_time;
}



void Pipeline::IdAsync::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_FATAL_STREAM("callback grf called. Not implemented for normal CommonTimedConstPtr messages yet.");
	///THis has to do the normal thing????

}
void Pipeline::IdAsync::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik)
{
	ROS_DEBUG_STREAM("callback ik called");
	auto newEvents1 = addEvent("id received ik ",message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback_real_wrenches"); 
	double filtered_t;
	addEvent("id: getting iks", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	addEvent("id: getting real wrenches", newEvents1);
	auto timestamp =message_ik->header.stamp;
	//ros::Time timestamp =convert_time_stamp_to_the_past(message_ik->header.stamp);
	auto grfs = Pipeline::IdAsync::get_wrench(timestamp);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	addEvent("calling run function of id", newEvents1);

	run(message_ik->header, filtered_t, iks, grfs, newEvents1);	
}

void Pipeline::IdAsync::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik) 

{
	ROS_DEBUG_STREAM("reached ik filtered subscriber. good news");
	auto newEvents1 = addEvent("id received ik filtered ", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	//cant find the right copy constructor syntax. will for loop it
	addEvent("id: getting iks already filtered.", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik);
	addEvent("id: getting real wrenches", newEvents1);
	auto timestamp =message_ik->header.stamp;
	//ros::Time timestamp =convert_time_stamp_to_the_past(message_ik->header.stamp);
	auto grfs = Pipeline::IdAsync::get_wrench(timestamp);
	addEvent("calling run function of id", newEvents1);
	ROS_DEBUG_STREAM("calling run function");
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}



