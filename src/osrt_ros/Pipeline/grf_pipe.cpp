#include "InverseDynamics.h"
#include "Ros/include/common_node.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/Event.h"
#include "opensimrt_msgs/Events.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <exception>
#include <numeric>
#include <utility>
#include <vector>
#include "ros/service_server.h"
#include "ros/time.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/grf_pipe.h"
#include "osrt_ros/events.h"
#include "opensimrt_bridge/conversions/message_convs.h"

using namespace std;
using namespace OpenSim;
;
using namespace OpenSimRT;

Pipeline::Grf::Grf()
{
	//TODO: Move common things here
	ROS_WARN_STREAM("not implemented");
}

Pipeline::Grf::~Grf()
{
	ROS_INFO_STREAM("Shutting down Grf");
}
void Pipeline::Grf::get_params()
{
	ros::NodeHandle nh("~");
	nh.param<std::string>("model_file", modelFile, "");
	ROS_INFO_STREAM("Using modelFile:" << modelFile);

}

void Pipeline::Grf::onInit() {
	Ros::CommonNode::onInit(true);
	Pipeline::Grf::get_params();
	previousTime = ros::Time::now().toSec();
	previousTimeDifference = 0;
	ros::NodeHandle nh("~");
	w_foot_l = WrenchPub(nh, "left"); //TODO:spaget, fix
	w_foot_l.onInit();
	w_foot_r = WrenchPub(nh, "right");
	w_foot_r.onInit();
	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	initializeLoggers("grfRight",grfRightLogger);
	initializeLoggers("grfLeft", grfLeftLogger);
	initializeLoggers("tau",tauLogger);
	
	ROS_INFO_STREAM("Setting up synchronized output for visualizers");
	ROS_WARN_STREAM("Outlabels are not implemented for these outputs, if there is reshuffling, they will not look correct.");
	sync_output = nh.advertise<opensimrt_msgs::Dual>("output_combined", 1);
	sync_output_filtered = nh.advertise<opensimrt_msgs::DualPos>("output_combined_filtered", 1);

}

void Pipeline::Grf::run(double t, SimTK::Vector q,SimTK::Vector qDot, SimTK::Vector qDDot, std_msgs::Header h, opensimrt_msgs::Events e) 
{
	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-3 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_DEBUG_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_DEBUG_STREAM("DeltaT :"<< std::setprecision (15) << t);

	/*if (t_old - t > 0.1)
	  ROS_ERROR("Reading from different timestamp! Did I lose a frame");
	  else // same timestamp, so we check the indexes are okay.
	  {
	  for (int it=0; it < qRaw.size(); it++)
	  {
	  if (qRaw[it] - qRaw_old[it] > 0.1)
	  ROS_ERROR("Difference too big");
	  }

	  }
	  */
	//	ROS_DEBUG_STREAM("TAN" << qRaw);

	// increment the time by the total simulation time plus the sampling
	// period, to keep increasing after each simulation loop
	//			t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);


	chrono::high_resolution_clock::time_point t1;
	t1 = chrono::high_resolution_clock::now();

	//TODO: add these events as well
	// perform grfm prediction
	detector->updDetector({t, q, qDot, qDDot});
	addEvent("grf updated detector",e);
	ROS_DEBUG_STREAM("Update detector ok");
	OpenSimRT::GRFMNonSmooth::Output grfmOutput = grfm->solve({t, q, qDot, qDDot});
	addEvent("grf solved GRFMNonSmooth estimation",e);
	ROS_DEBUG_STREAM("GRFM estimation ran ok");

	chrono::high_resolution_clock::time_point t2;
	t2 = chrono::high_resolution_clock::now();
	sumDelayMS +=
		chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
	sumDelayMSCounter++;

	// project on plane
	grfmOutput.right.point =
		projectionOnPlane(grfmOutput.right.point, grfOrigin);
	grfmOutput.left.point =
		projectionOnPlane(grfmOutput.left.point, grfOrigin);

	// setup ID inputn
	ExternalWrench::Input grfRightWrench = {grfmOutput.right.point,
		grfmOutput.right.force,
		grfmOutput.right.torque};
	ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
		grfmOutput.left.force,
		grfmOutput.left.torque};

	//TODO: conside if ID here is necessary
	addEvent("grf before id",e);
	// solve ID
	OpenSimRT::InverseDynamics::Output idOutput;
	bool do_inverse_dynamics = false;
	if (do_inverse_dynamics)
		idOutput = id->solve(
			{t, q, qDot, qDDot,
			vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

	ROS_DEBUG_STREAM("inverse dynamics ran ok");
	//ROS_INFO_STREAM("t: ["<< t << "] q: [" << q << "] tau: [" << idOutput.tau << "]");
	addEvent("grf after id", e);
	// visualization
	try {
		visualizer->update(q);
		ROS_DEBUG_STREAM("updated visuals ok");
		rightGRFDecorator->update(grfmOutput.right.point,
				grfmOutput.right.force);
		leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
		addEvent("grf after visualizer", e);
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}

	//
	//OpenSim::TimeSeriesTable output;

	opensimrt_msgs::CommonTimed msg = Osb::get_GRFMs_as_common_msg(grfmOutput,t,h);
	msg.events = e;
	pub.publish(msg);
	w_foot_r.publish(h, grfRightWrench);
	w_foot_l.publish(h, grfLeftWrench);

	//setting up synchronized output
	opensimrt_msgs::CommonTimed msg_ik;
	opensimrt_msgs::PosVelAccTimed msg_ik_filtered = Osb::get_as_ik_filtered_msg(h, t, q, qDot, qDDot);
	
	Osb::update_pose(msg_ik,t,q);
        opensimrt_msgs::Dual dual_msg;
	dual_msg.q = msg_ik;
	dual_msg.tau = msg;
	sync_output.publish(dual_msg);
        opensimrt_msgs::DualPos dual_filtered_msg;
	dual_filtered_msg.qqq = msg_ik_filtered;
	dual_filtered_msg.tau = msg;
	sync_output_filtered.publish(dual_filtered_msg);
	//added publisher for the already synchronized msgs.
	try{
		// log data (use filter time to align with delay)
		if(isRecording())
		{
			//ROS_WARN_STREAM("THIS SHOULDNT BE RUNNING");
			grfRightLogger->appendRow(grfmOutput.t, ~grfmOutput.right.toVector());
			grfLeftLogger->appendRow(grfmOutput.t, ~grfmOutput.left.toVector());
			tauLogger->appendRow(t, ~idOutput.tau);
			ROS_INFO_STREAM("Added data to loggers. "<< counter);
		}
	}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
	//if (counter > 720)
	//	write_();
	//}




}



void Pipeline::Grf::callback(const opensimrt_msgs::CommonTimedConstPtr& message) {
	auto newEvents = addEvent("grf received ik", message);
	ROS_DEBUG_STREAM("Received message. Running Grf loop"); 
	counter++;
	SimTK::Vector qRaw(message->data.size()); //cant find the right copy constructor syntax. will for loop it
	for (int j = 0;j < qRaw.size();j++)
	{
		qRaw[j] = message->data[j];
	}

	double t = message->time;
	// filter
	auto ikFiltered = filter->filter({t, qRaw});
	auto q = ikFiltered.x;
	auto qDot = ikFiltered.xDot;
	auto qDDot = ikFiltered.xDDot;
	ROS_DEBUG_STREAM("Filter ran ok");
	if (!ikFiltered.isValid) {
		ROS_DEBUG_STREAM("filter results are NOT valid");
		return; }
	ROS_DEBUG_STREAM("Filter results are valid");

	run(ikFiltered.t, q, qDot, qDDot,message->header, newEvents);	
}	
void Pipeline::Grf::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message) {
	auto newEvents = addEvent("grf received ik", message);
	ROS_DEBUG_STREAM("Received message. Running Grf filtered loop"); 
	counter++;
	//cant find the right copy constructor syntax. will for loop it
	SimTK::Vector q(message->d0_data.size()),qDot(message->d1_data.size()),qDDot(message->d2_data.size()); 
	for (int j = 0;j < q.size();j++)
	{
		q[j] = message->d0_data[j];
		qDot[j] = message->d1_data[j];
		qDDot[j] = message->d2_data[j];
	}
	run(message->time, q, qDot, qDDot,message->header, newEvents);

}	
void Pipeline::Grf::finish() {
	cout << "Mean delay: " << double(sumDelayMS) / sumDelayMSCounter << " ms"
		<< endl;

	// Relax tolerance because of floating point errors between target machines
	// (this fails on Windows).
	OpenSimUtils::compareTables(
			*grfRightLogger,
			TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
				"acceleration_based/wrench_right.sto"),
			1e-1);
	OpenSimUtils::compareTables(
			*grfLeftLogger,
			TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
				"acceleration_based/wrench_left.sto"),
			1e-1);
	OpenSimUtils::compareTables(
			*tauLogger,
			TimeSeriesTable(
				subjectDir +
				"real_time/grfm_prediction/acceleration_based/tau.sto"),
			1e-1);

}
bool Pipeline::Grf::see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
	std::copy(grfRightLogger->getTableMetaData().getKeys().begin(), grfRightLogger->getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
	ss << std::endl;
	ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());
	return true;
}
void Pipeline::Grf::write_() {
	//			std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
	//		  	std::copy(grfRightLogger.getTableMetaData().getKeys().begin(), grfRightLogger.getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
	//		  	ss << std::endl;
	//			ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());


	/*	ROS_INFO_STREAM("I TRY WRITE sto");
		STOFileAdapter::write(grfRightLogger,"grfRight.sto");
		STOFileAdapter::write(grfLeftLogger,"grfLeft.sto");
		STOFileAdapter::write(tauLogger,"tau.sto");
		ROS_INFO_STREAM("I TRY WRITE csv");
		CSVFileAdapter::write(grfRightLogger,"grfRight.csv");
		CSVFileAdapter::write(grfLeftLogger,"grfLeft.csv");
		CSVFileAdapter::write(tauLogger,"tau.csv");
		*/
	saveCsvs();
	saveStos();
	ROS_INFO_STREAM("i write");
}

