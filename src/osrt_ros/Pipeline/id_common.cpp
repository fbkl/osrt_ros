#include "InverseDynamics.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "opensimrt_msgs/OpenSimData.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "message_filters/time_synchronizer.h"
#include "osrt_ros/parameters.h"
#include "ros/duration.h"
#include "ros/exception.h"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>
#include <SimTKcommon/internal/Vector_.h>
#include <Simulation/Model/BodySet.h>
#include <Simulation/Model/PhysicalFrame.h>
#include <Simulation/SimbodyEngine/Body.h>
#include <exception>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include "ros/service_server.h"
#include "ros/time.h"
#include "signal.h"
#include "std_srvs/Empty.h"

#include "osrt_ros/Pipeline/id_common.h"
#include "opensimrt_bridge/conversions/message_convs.h"
#include "tf2/convert.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::IdCommon::IdCommon(): Pipeline::DualSink::DualSink(false)
{
	//TODO: this needs to be abstracted. I have this copied over and over again. maybe that should be done before standardizing
	ROS_DEBUG_STREAM("Called IdCommon constructor.");
	std::string modelFile = "";
	nh.param<std::string>("model_file",modelFile,"");
	nh.param<bool>("use_grfm_filter", use_grfm_filter, false);
	//TODO:params!!!! copy from bridge

	// setup model
	Object::RegisterType(Thelen2003Muscle());
	ROS_INFO_STREAM("Using model: " << modelFile);
	model = new Model(modelFile);
	OpenSimUtils::removeActuators(*model);
	model->initSystem();

	// setup external forces
	//Storage grfMotion(grfMotFile);

	grfRightFootPar = pars::getparamWrench(nh, "grf_right");
	grfRightLabels = pars::getparamGRFMLabels(nh, "grf_right");

	grfLeftFootPar = pars::getparamWrench(nh, "grf_left");
	grfLeftLabels = pars::getparamGRFMLabels(nh, "grf_left");
	
	wrenchParameters.push_back(grfRightFootPar);
	wrenchParameters.push_back(grfLeftFootPar);

	// setup filters
	ROS_DEBUG_STREAM("setting up filters");
	LowPassSmoothFilter::Parameters filterParam;
	if (pars::getparamFilterIK(nh, model->getNumCoordinates(), filterParam))
		ikfilter = new LowPassSmoothFilter(filterParam);
	else
		ROS_WARN_STREAM("IK Filter not created, if used this will crash.");
	
	LowPassSmoothFilter::Parameters grfFilterParam = pars::getparamFilterGRFM(nh);

	//grffilter = new LowPassSmoothFilter(grfFilterParam);
	// test with state space filter
	// StateSpaceFilter ikFilter({model.getNumCoordinates(), cutoffFreq});
	// StateSpaceFilter grfRightFilter({9, cutoffFreq}), grfLeftFilter({9,
	// cutoffFreq});
	ROS_DEBUG_STREAM("filters set up ok.");

	// initialize id and logger
	id = new InverseDynamics(*model, wrenchParameters);
	ROS_INFO_STREAM("Setting loggers,");
	OpenSim::TimeSeriesTable tauLoggerTemp = id->initializeLogger();
	tauLogger = new TimeSeriesTable();
	output.labels = tauLoggerTemp.getColumnLabels();
	tauLogger->setColumnLabels(tauLoggerTemp.getColumnLabels());
	// now the loggers for IK and grfms:
	ikLogger = new TimeSeriesTable();
	grfmRightLogger = new TimeSeriesTable();
	grfmLeftLogger = new TimeSeriesTable();
	ROS_INFO_STREAM("loggers created!");

}

Pipeline::IdCommon::~IdCommon()
{
	ROS_INFO_STREAM("Shutting down Id");
}

void Pipeline::IdCommon::print_vec(std::vector<std::string> vs)
{
	std::string s="[";
	for (auto vsi:vs)
		s+= vsi+", ";
	s+= "]";
	ROS_INFO_STREAM(s);
}

void Pipeline::IdCommon::onInit() {
	//TODO: this is technically wrong. if I am subscribing to the version with CommonTimed version, then I definetely want the second label as well, but it will fail if I am not subscribing to this, so this flag needs to be set only in that case
	nh.getParam("get_second_label", get_second_label);
	//get_second_label = false;
	Pipeline::DualSink::onInit();

	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	sub0.registerCallback(&Pipeline::IdCommon::callback0,this);
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	sub1.registerCallback(&Pipeline::IdCommon::callback1,this);


	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	// TODO: set the column labels, or it will break when you try to use them!
	message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync(sub, sub2, 500);
	sync.registerCallback(std::bind(&Pipeline::IdCommon::callback, this, std::placeholders::_1, std::placeholders::_2));
	sync.registerCallback(&Pipeline::IdCommon::callback, this);

	//i should have the input labels from grf already
	if(get_second_label)
	{
		//TODO:: this looks like a reimplementation of the Reshuffler idea. check and fix!
		ROS_INFO_STREAM("left");
		/*for (auto l:grfLeftLabels)
			ROS_DEBUG_STREAM(l <<";");
		ROS_DEBUG_STREAM("input2_labels");
		for (auto il:input2_labels)
			ROS_DEBUG_STREAM(il << ";");
		*/
		grfLeftIndexes = Osb::generateIndexes(grfLeftLabels,input2_labels);
		ROS_INFO_STREAM("right");
		grfRightIndexes = Osb::generateIndexes(grfRightLabels, input2_labels);
	}
	//CRAZY DEBUG
	if (input.labels.size() == 0)
	{
		ROS_FATAL_STREAM("cannot set ik logger! input labels is empty!!!!");
		throw std::runtime_error("input labels not set. cannot initiate ikLogger");
	}
	ikLogger->setColumnLabels(input.labels);
	if (input2_labels.size() == 0)
	{
		//ROS_FATAL_STREAM("cannot set grmflogger! input2_labels is empty");
		//throw std::runtime_error("input2_labels not set. cannot start grfm loggers");
		//but I have the identifiers right?
		//actually this is the easy version, the other type I will need to think 	
		grfmRightLogger->setColumnLabels(grfRightLabels);
		grfmLeftLogger->setColumnLabels(grfLeftLabels);
	}
	else
	{

	ROS_WARN_STREAM("NOT YET IMPLEMENTED: if I reach this, then I need to use not the version from params, but instead figure out from custom labels from the service call. Doesnt sound very hard, probably just cut it in half and put one on one side and the other in the other side, but sides can change and maybe what I want to do with this is use multiple GRFM inputs, say with multiple force plates. In this case none if this will work. grmf input labels::");
	for (auto a_label:input2_labels)
	{
		ROS_WARN_STREAM("input2_labels:"<<a_label);
	}
	ROS_WARN_STREAM("end of input2_labels.");
	}

	ROS_INFO_STREAM("Attempting to set loggers.");
	//I put the prefix here which tells me it is from ID, so I wont get confused where it comes from. 
	initializeLoggers("grfRight",grfmRightLogger);
	initializeLoggers("grfLeft", grfmLeftLogger);
	initializeLoggers("ik",ikLogger);
	print_vec(tauLogger->getColumnLabels());

	initializeLoggers("tau",tauLogger);


}




void Pipeline::IdCommon::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) {
	ROS_INFO_STREAM("callback ik called");
}
void Pipeline::IdCommon::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_INFO_STREAM("callback grf called");
}

void Pipeline::IdCommon::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback."); 
	double filtered_t;
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	//ROS_DEBUG_STREAM("message_grf\n" << *message_grf);
	auto grfs = Osb::get_wrench(message_grf, grfRightIndexes, grfLeftIndexes);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	run(message_ik->header, filtered_t, iks, grfs, bothEvents);	
}



void Pipeline::IdCommon::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop"); 
	//cant find the right copy constructor syntax. will for loop it
	auto iks = Osb::parse_ik_message(message_ik);
	auto grfs = Osb::get_wrench(message_grf, grfRightIndexes, grfLeftIndexes);
	run(message_ik->header, message_ik->time, iks,grfs, bothEvents);

}	


void Pipeline::IdCommon::run(const std_msgs::Header h , double t, std::vector<SimTK::Vector> iks, std::vector<ExternalWrench::Input> grfs, opensimrt_msgs::Events e ) 

{
	//ROS_INFO_STREAM("here0");;
	ROS_DEBUG_STREAM("Received run call. Running Id run loop.");	    
	//ROS_INFO_STREAM("time diff" << h.stamp-last_received_ik_stamp);
	//ROS_INFO_STREAM("this stamp" << h.stamp << "last_received_ik_stamp" << last_received_ik_stamp);
	//unpacks wrenches:

	if(grfs.size() == 0)
	{
		ROS_WARN_STREAM("THERE ARE NO GRFS!");
		return;
	}
	auto grfLeftWrench = grfs[0]; 
	auto grfRightWrench = grfs[1]; 

	//filter wrench!
	//TODO
	if(iks.size() == 0)
	{
		ROS_WARN_STREAM("THERE ARE NO IKS!");
		return;
	}
	auto q = iks[0];
	auto qDot = iks[1];
	auto qDDot = iks[2];
	//
	// perform id
	chrono::high_resolution_clock::time_point t1;
	t1 = chrono::high_resolution_clock::now();
	addEvent("id_normal before id", e);

	//ROS_INFO_STREAM("Going to run ID");;
	auto idOutput = id->solve(
			{t, q, qDot, qDDot,
			vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});
	addEvent("id_normal after id",e);

	ROS_DEBUG_STREAM("ID ran ok");
	//ROS_INFO_STREAM("here2");;

	try
	{
	//ROS_INFO_STREAM("here4");;
	

		//TODO: this is common for everyone that uses common messages, make it a class
		//I need the header!!!
		opensimrt_msgs::CommonTimed msg;
		msg.header = h;
		//ROS_DEBUG_STREAM("attempting to print tau!");
		for (double tau_component:idOutput.tau)
		{
			//ROS_DEBUG_STREAM("some_tau_component: " << tau_component);
			msg.data.push_back(tau_component);
		}
		msg.events = e;
		pub.publish(msg);
		std::vector<OpenSimRT::ExternalWrench::Input> wV;
		wV.push_back(grfLeftWrench);
		wV.push_back(grfRightWrench);


		publish_additional_topics(h, q, wV );

		std::vector<opensimrt_msgs::OpenSimData> osdv;
		opensimrt_msgs::OpenSimData ik_data, tau_data;
		
		ik_data.data.insert(ik_data.data.end(), iks[0].begin(), iks[0].end());
		osdv.push_back(ik_data);
		tau_data.data.insert(tau_data.data.end(), idOutput.tau.begin(), idOutput.tau.end());
		osdv.push_back(tau_data);
		osdv.push_back(Osb::reverse_parse_wrench(grfLeftWrench));
		osdv.push_back(Osb::reverse_parse_wrench(grfRightWrench));
		

		addEvent("id_common: before publish multi",e);
		opensimrt_msgs::MultiMessage more_complete= Osb::get_as_Multi(h,osdv,e.list);
		more_complete.time = t;
		/*pub_grf_left.publish(conv_grf_to_msg(h, grfLeftWrench));
		pub_grf_right.publish(conv_grf_to_msg(h, grfRightWrench));
		pub_ik.publish(conv_ik_to_msg(h, q));
		last_received_ik_stamp = msg.header.stamp;*/
		////old version
		//sync_output_multi.publish(Osb::get_as_Multi(h,t,q,idOutput.tau,e.list));
		sync_output_multi.publish(more_complete);
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR_STREAM("Ros error while trying to publish ID output: " << e.what());
	}
	try{

		// log data (use filter time to align with delay)
		if(isRecording())
		{
			tauLogger->appendRow(t, ~idOutput.tau);
			ikLogger->appendRow(t,~q);
			grfmLeftLogger->appendRow(t,~grfLeftWrench.toVector());
			grfmRightLogger->appendRow(t,~grfRightWrench.toVector());
		}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
	//ROS_INFO_STREAM("here5");;
	//if (counter > 720)
	//	write_();
	//}
}	

void Pipeline::IdCommon::finish() {
	ROS_ERROR_STREAM("deprecated.");
}
bool Pipeline::IdCommon::see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_ERROR_STREAM("deprecated.");
	return true;
}
void Pipeline::IdCommon::write_() {
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
