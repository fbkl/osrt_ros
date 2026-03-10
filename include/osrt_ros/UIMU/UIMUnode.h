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

class GetPointFromSomeTF
{
	public:
		//tf::TransformListener tl;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;
		std::vector<std::string> markerNames;
		std::vector<std::string> tfNames;
		ros::NodeHandle nh{"~/marker"};
		std::unordered_map<std::string, std::string> markerDefList;
		std::unordered_map<std::string, geometry_msgs::TransformStamped> latest_marker_tfs;
		std::string tf_frame_prefix;
		std::string world_tf_reference;
		double tf_timeout;
		GetPointFromSomeTF(): tfListener(tfBuffer) 
	{

		nh.param<double>("tf_timeout",tf_timeout,0.05);
		nh.param<std::string>("world_tf_reference",world_tf_reference,"map");
		nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
		try{	
			XmlRpc::XmlRpcValue markerList;
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
				ROS_INFO_STREAM("AR: parsing points and tf map");
				ROS_INFO_STREAM("AR: Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
				for (int32_t i = 0; i < markerList.size(); ++i) 
				{
					ROS_INFO("AR: Entered loop");
					XmlRpc::XmlRpcValue markerDef = markerList[i]; 
					ROS_INFO("AR: Loaded MarkerDef");
					std::string this_marker_name = markerDef["marker_name"];
					ROS_INFO_STREAM("AR: Assigned name: " << this_marker_name);
					markerNames.push_back(this_marker_name);
					ROS_INFO_STREAM("AR: ADDED TO MARKER LIST: " << magenta << markerNames.back() );
					std::string this_marker_tf   = tf_frame_prefix + "/" + (std::string)markerDef["marker_tf"];
					ROS_INFO_STREAM("AR: assigned tf: " << this_marker_tf);
					markerDefList[this_marker_name] = this_marker_tf;
					ROS_INFO("AR: Assigned to map");
					tfNames.push_back(this_marker_tf);
					ROS_INFO("AR: ADDED TO TF LIST");
					//We also add the default position of the marker so that if the AR is broken, it doesnt affect the measurement too much
					XmlRpc::XmlRpcValue this_marker_default_position = markerDef["default_position"];
					ROS_INFO_STREAM(cyan <<"read default_position ok"<<reset);
					ROS_ASSERT(this_marker_default_position.getType() == XmlRpc::XmlRpcValue::TypeArray); //
					double x = this_marker_default_position[0];
					double y = this_marker_default_position[1];
					double z = this_marker_default_position[2];
					ROS_INFO_STREAM(cyan <<"setting x,y,z okay"<<reset);
					geometry_msgs::TransformStamped transform;
					transform.transform.translation.x = x;
					transform.transform.translation.y = y;
					transform.transform.translation.z = z;
					latest_marker_tfs[this_marker_tf] = transform;


					ROS_INFO_STREAM(cyan <<"FINISHED SETTING UP ONE MARKER AT LEAST"<<reset);
					//ROS_ASSERT(markerDef[i].getType() == XmlRpc::XmlRpcValue::TypeString);
				}
				//for (auto& marker:markerNames)
				//	marker+=tf_frame_prefix;
			}
		}
		catch(XmlRpc::XmlRpcException& e)
		{
			ROS_ERROR_STREAM("AR: Could not setup markers" << e.getMessage());
		}
		ROS_INFO("AR: Finished serring up markers");

	}

		SimTK::Array_<SimTK::Vec3> get_translations()
		{
			SimTK::Array_<SimTK::Vec3> markerObservations;

			for (const auto& [this_marker_name, this_marker_tf] : markerDefList)
			{
				SimTK::Vec3 v;
				try{
					geometry_msgs::TransformStamped transform;
					//transform = tfBuffer.lookupTransform( this_marker_tf, world_tf_reference, ros::Time(0), ros::Duration(tf_timeout) ); //
					transform = tfBuffer.lookupTransform( world_tf_reference, this_marker_tf, ros::Time(0), ros::Duration(tf_timeout) ); //
																			     //transform = tfBuffer.lookupTransform( "map", this_marker_tf, ros::Time(0), ros::Duration(tf_timeout) ); //
					latest_marker_tfs[this_marker_tf] = transform;
				}
				catch (tf::TransformException& ex){
					ROS_ERROR_THROTTLE(60,"AR: Translation part Transform exception! %s",ex.what());
				}
				auto transform = latest_marker_tfs[this_marker_tf];
				//if you are in a hurry just hard code the transform here because we just want it to work now.
				v.set(0, transform.transform.translation.x);
				v.set(1, transform.transform.translation.y);
				v.set(2, transform.transform.translation.z);
				markerObservations.push_back(v);

			}

			return markerObservations;
		}
};


class UIMUnode: Ros::CommonNode
{
	public:
		UIMUnode(): Ros::CommonNode(false) //if true debugs
						   //UIMUnode(): Ros::CommonNode()
	{}
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1;
		std::vector<std::string> imuObservationOrder;
		double rate;
		ros::Rate* r;
		std::string modelFile;
		std::string loggerFileNameIK,loggerFileNameIMUs;
		std::string tf_frame_prefix;
		double sumDelayMS = 0, numFrames = 0;
		double previousTime = 0;
		double previousDt = 0;
		OpenSim::TimeSeriesTable imuLogger, imuCalibrationLogger, qRawLogger, qLogger, qDotLogger, qDDotLogger;

		ros::Publisher time_pub, time_ik_pub;
		ros::ServiceServer calibrationService;
		UIMUInputDriver *driver;
		InverseKinematics * ik;
		IMUCalibrator * clb;
		bool clb_is_ready =false;
		bool usePositionMarkers;
		bool visualiseIt= false;
		//ros::Publisher re_pub;
		//filter parameters
		double cutoffFreq;
		int splineOrder, memory, delay;
		OpenSimRT::LowPassSmoothFilter * ikfilter;
		std::vector<ros::Publisher> plottable_outputs;
		//dynamic_reconfigure::Server<osrt_ros::UIMUConfig> server;
		//dynamic_reconfigure::Server<osrt_ros::UIMUConfig>::CallbackType f;
		OpenSim::Model model;

		GetPointFromSomeTF* tfPointGetter;


		void get_params()
		{
			ros::NodeHandle nh("~");
			nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
			// imu calibration settings

			nh.param<std::string>("imu_direction_axis", imuDirectionAxis, "");

			nh.param<std::string>("imu_base_body", imuBaseBody, "");

			nh.param<double>("imu_ground_rotation_x", xGroundRotDeg1, 0.0);
			nh.param<double>("imu_ground_rotation_y", yGroundRotDeg1, 0.0);
			nh.param<double>("imu_ground_rotation_z", zGroundRotDeg1, 0.0);
			nh.getParam("imu_observation_order", imuObservationOrder);
			if (imuObservationOrder.size() == 0)
			{
				ROS_FATAL("IMU observation order not defined!");
				throw(std::invalid_argument("imuObservationOrder not defined."));
			}
			else
			{
				ROS_INFO_STREAM("Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
			}


			// driver send rate
			nh.param<double>("rate", rate, 0.0);
			r = new ros::Rate(rate);
			nh.param<bool>("visualise", visualiseIt, true);
			// subject data
			nh.param<std::string>("model_file", modelFile, "");
			ROS_INFO_STREAM("Using modelFile:" << modelFile);
			nh.param<std::string>("logger_filename_ik", loggerFileNameIK, "test_ik");
			nh.param<std::string>("logger_filename_imus", loggerFileNameIMUs, "test_imus");

			nh.param<bool>("use_position_markers", usePositionMarkers, false);

			nh.param<bool>("filter_output",publish_filtered, true);
			nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
			nh.param<int>("memory", memory, 0);
			nh.param<int>("spline_order", splineOrder, 0);
			nh.param<int>("delay", delay, 0);

			if( usePositionMarkers)
			{
				ROS_INFO_STREAM("Also using position Markers!");
				tfPointGetter = new GetPointFromSomeTF;
			}

			ROS_DEBUG_STREAM("Finished getting params.");


		}
		void registerType(Object* muscleModel) //do I even need this?
		{

			Object::registerType(*muscleModel);

		}
		void reconfigure_callback(osrt_ros::UIMUConfig &config, uint32_t level){
			ROS_INFO("IMU Ground Orientation reconfigure request %s, (%f, %f, %f)", config.imu_direction_axis_param.c_str(), config.imu_ground_rotation_x, config.imu_ground_rotation_y,config.imu_ground_rotation_z);

			if (clb_is_ready)
			{
				imuDirectionAxis = config.imu_direction_axis_param;
				xGroundRotDeg1 = config.imu_ground_rotation_x;
				yGroundRotDeg1 = config.imu_ground_rotation_y;
				zGroundRotDeg1 = config.imu_ground_rotation_z;
				start_ik();
			}
			else
				ROS_WARN("IMU Ground Orientation reconfigure request warning: calibrator not yet defined.");

		}
		void reconfigure_heading_callback(osrt_ros::headingConfig &config, uint32_t level){
			ROS_INFO("Heading Reconfigure request base IMU heading angle: %f", config.base_imu_heading);

			if (clb_is_ready)
			{
				clb->baseHeadingAngle = config.base_imu_heading;
				//I need to change the things that are related to the heading here!
				start_ik();

			}
			else
				ROS_WARN("Heading Reconfigure request warning: calibrator not yet defined.");

		}
		vector<InverseKinematics::MarkerTask> markerTasks;
		vector<InverseKinematics::IMUTask> imuTasks;
		void define_tasks()
		{

			// marker tasks
			ROS_DEBUG_STREAM("Setting up markerTasks");
			if (usePositionMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here.
			{
				vector<string> markerObservationOrder;
				for (auto some_marker_name:tfPointGetter->markerNames)
					ROS_WARN_STREAM("AR positional marker name: "<<some_marker_name);

				InverseKinematics::createMarkerTasksFromMarkerNames(model, tfPointGetter->markerNames, markerTasks,
						markerObservationOrder);
			}

			// imu tasks
			ROS_DEBUG_STREAM("Setting up imuTasks");
			InverseKinematics::createIMUTasksFromObservationOrder(
					model, imuObservationOrder, imuTasks);

			ROS_DEBUG_STREAM("Starting driver");
			{
				string imuObservationOrderStr;
				for(auto a:imuObservationOrder)
				{
					imuObservationOrderStr +=a+", ";
				}
				ROS_INFO_STREAM("Using imu observation order: " << imuObservationOrderStr);
			}
		}
		void start_ik()
		{
			chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;
			ROS_DEBUG_STREAM("setGroundOrientationSeq");
			if (false)
			{
				clb->R_GoGi1 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
				ROS_INFO("Setting ground orientation from params");
			}
			else
			{
				auto R_GoGi2 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
				Vec3 trans_p{1,1,1};
				Vec3 trans_p2{1.1,1,1};
				SimTK::Transform TX(R_GoGi2,trans_p);
				SimTK::Transform TX2(~R_GoGi2,trans_p2);
				clb->sameHeader.stamp = ros::Time::now();
				clb->publishTransform("imu_R_GoGi_original", TX, clb->sameHeader);
				clb->publishTransform("imu_R_GiGo_original", TX2, clb->sameHeader);
				Vec3 trans_p0{1.1,1,-1};
				clb->R_GoGi1 = ~clb->setGroundOrientationFromTF("imu_ref_ori");
				SimTK::Transform TX0(~clb->R_GoGi1,trans_p0);
				clb->publishTransform("imu_ref_ori_inv", TX0, clb->sameHeader);
				ROS_WARN_STREAM("UNTESTED!!! setting ground orientation from TF what i defined:"<< clb->R_GoGi1 << "\nwhat was before" << R_GoGi2 );
			}
			ROS_DEBUG_STREAM("heading");
			clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis);

			//std::cout << boost::stacktrace::stacktrace() << std::endl;
			clb->calibrateIMUTasks(imuTasks);
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");

			// initialize ik (lower constraint weight and accuracy -> faster tracking)
			ROS_DEBUG_STREAM("Setting up IK");
			ik = new InverseKinematics(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
			qRawLogger = ik->initializeLogger();
			initializeLoggers(loggerFileNameIK,&qRawLogger);

			//TODO: publish correct ROS topics
			output.labels = qRawLogger.getColumnLabels();
			ROS_INFO_STREAM("Done with start_ik");
			chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;

			ROS_WARN_STREAM(bar << "start_ik call duration in ms:"<<magenta<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count()<<bar <<reset);
		}

		SimTK::RowVector fromVectorOfSimTKQuaternionsToARowVector(std::vector<SimTK::Quaternion> vv)
		{
			std::vector<double> serialized;

			for (auto q:vv)
			{
				ROS_INFO_STREAM("Weird serializer (quaternion): " << q[0] <<","<<q[1]<<","<<q[2]<<","<<q[3]);
				serialized.push_back(q[0]);
				serialized.push_back(q[1]);
				serialized.push_back(q[2]);
				serialized.push_back(q[3]);
			}

			SimTK::RowVector serializedv(serialized.size());
			for (int ii = 0; ii <serializedv.size(); ii++)
			{
				serializedv[ii] = serialized[ii];
			}
			return serializedv;
		}
		void clearLogger(TimeSeriesTable &t) //TODO: move it somewhere nice. maybe make loggers a wrapper class
		{
			for (size_t iii= 0; iii<t.getNumRows();iii++)
				t.removeRow(0);
			if (t.getNumRows() == 0)
				ROS_INFO_STREAM("logger cleared");
			else
				ROS_WARN_STREAM("couldnt clear logger table!");
		}
		void doCalibrate()
		{
			ROS_DEBUG_STREAM("clb samples");
			clb->recordNumOfSamples(100); //TODO:PARAM!
			clb_is_ready = true;
			clearLogger(imuCalibrationLogger);
			ROS_INFO_STREAM("After clearing table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
			imuCalibrationLogger.appendRow(0, fromVectorOfSimTKQuaternionsToARowVector(clb->staticPoseQuaternions)); //if this is the time, maybe we want to add the calibration time here as well. Also, maybe we don't want to clear the calibration, or clear only after saving? TODO: think about this
			ROS_INFO_STREAM("After appending clb samples to table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
		}
		bool calibrationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
		{
			ROS_INFO_STREAM("Calibration service called!");
			doCalibrate();
			//I need to restart ik again as well
			start_ik();
			return true;
		}
		void onInit()

		{
			get_params();
			//f = boost::bind(&UIMUnode::reconfigure_callback, this, _1,_2);
			//server.setCallback(f);
			time_pub = nh.advertise<std_msgs::Int64>("time",1);
			time_ik_pub = nh.advertise<std_msgs::Int64>("time_ik",1);
			calibrationService = nh.advertiseService("calibrate", &UIMUnode::calibrationSrv, this);
			// setup model
			ROS_DEBUG_STREAM("Setting up model.");
			model = OpenSim::Model(modelFile);
			OpenSimUtils::removeActuators(model);

			ROS_DEBUG_STREAM("Staring UIMUInputDriver with tf_frame_prefix" << tf_frame_prefix << " and rate: " << rate );
			driver = new UIMUInputDriver(imuObservationOrder,tf_frame_prefix,rate); //uses tf server
			driver->startListening();
			imuLogger = driver->initializeLogger();
			initializeLoggers(loggerFileNameIMUs,&imuLogger);
			imuCalibrationLogger = driver->initializeCalibrationValuesLogger();
			initializeLoggers("calib", &imuCalibrationLogger);

			// calibrator
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");
			clb = new IMUCalibrator(model, driver, imuObservationOrder);


			doCalibrate();

			define_tasks();
			start_ik();

			string all_labels;
			ros::NodeHandle nh("~");
			ROS_INFO_STREAM("Setting plottable_outputs topic advertisers...");
			for (auto l:output.labels)
			{

				plottable_outputs.push_back(nh.advertise<std_msgs::Float64>("joints/"+l,1));
				all_labels+=l+", ";
			}
			ROS_INFO_STREAM("Publisher labels: "<<all_labels);
			//I want to start the service after we set the labels, otherwise it might reply with an empty message.
			Ros::CommonNode::onInit(0); //we are not reading from anything, we are a source

			if(publish_filtered)
			{
				//filter
				ROS_DEBUG_STREAM("Setting up filter");
				LowPassSmoothFilter::Parameters ikFilterParam;
				ikFilterParam.numSignals = model.getNumCoordinates();
				ikFilterParam.memory = memory;
				ikFilterParam.delay = delay;
				ikFilterParam.cutoffFrequency = cutoffFreq;
				ikFilterParam.splineOrder = splineOrder;
				ikFilterParam.calculateDerivatives = true;
				ROS_DEBUG_STREAM("filter parameters set.");
				ikfilter = new LowPassSmoothFilter(ikFilterParam);
				// initialize filtered loggers
				ROS_DEBUG_STREAM("getting columnNames from model");
				auto columnNames = OpenSimRT::OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
				string columnNamesStr = "";
				for(auto cn:columnNames)
				{
					columnNamesStr+=cn+", ";
				}
				ROS_DEBUG_STREAM("got columnNamesStr: " << columnNamesStr);
				qLogger.setColumnLabels(columnNames);
				qDotLogger.setColumnLabels(columnNames);
				qDDotLogger.setColumnLabels(columnNames);
				ROS_DEBUG_STREAM("columnNames for loggers set.");
				initializeLoggers("qLogger",&qLogger);
				initializeLoggers("qDotLogger",&qDotLogger);
				initializeLoggers("qDDotLogger",&qDDotLogger);

			}
			// mean delay
			ROS_DEBUG_STREAM("onInit finished just fine.");
		}

		void run() {
			try { // main loop
				while (!driver->shouldTerminate()) {
					opensimrt_msgs::CommonTimed msg;
					std_msgs::Header h;
					h.stamp = ros::Time::now();
					h.frame_id = "subject";
					msg.header = h;

					// get input from imus
					ROS_DEBUG_STREAM("Getting frame:");
					auto imuData = driver->getFrame();

					SimTK::Array_<SimTK::Vec3> markerObservations;
					if (usePositionMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here.
					{
						markerObservations = tfPointGetter->get_translations();

					}

					ROS_DEBUG_STREAM("Solving inverse kinematics:" );
					numFrames++;

					// solve ik
					chrono::high_resolution_clock::time_point t1;
					t1 = chrono::high_resolution_clock::now();

					auto pose = ik->solve(
							{imuData.first, markerObservations, clb->transform(imuData.second)});

					addEvent("ik",msg);
					chrono::high_resolution_clock::time_point t2;
					t2 = chrono::high_resolution_clock::now();
					sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
						.count();
					ROS_DEBUG_STREAM( "pose is:" << pose.q);

					//msg.data.push_back(pose.t);
					Osb::update_pose(msg, pose.t, pose.q);
					double Dt = pose.t-previousTime;
					double jitter = Dt-previousDt;

					ROS_DEBUG_STREAM("jitter(us):" << jitter*1000000);
					ROS_DEBUG_STREAM("delta_t   :" << Dt);
					ROS_DEBUG_STREAM("T (pose.t):" << pose.t);

					int i = 0;
					if (false)
						for (double joint_angle:pose.q)
						{
							ROS_DEBUG_STREAM("some joint_angle: "<<joint_angle << " will be sent to topic: " << plottable_outputs[i].getTopic());
							std_msgs::Float64 j_msg;
							j_msg.data = joint_angle*180/3.14159265;
							plottable_outputs[i].publish(j_msg);
							i++;
						}

					pub.publish(msg);
					if(publish_filtered)
					{
						auto ikFiltered = ikfilter->filter({pose.t, pose.q});
						auto q = ikFiltered.x;
						auto qDot = ikFiltered.xDot;
						auto qDDot = ikFiltered.xDDot;
						ROS_DEBUG_STREAM("Filter ran ok");
						if (!ikFiltered.isValid) {
							ROS_DEBUG_STREAM("filter results are NOT valid");
							continue; }
						ROS_DEBUG_STREAM("Filter results are valid");
						opensimrt_msgs::PosVelAccTimed msg_filtered = Osb::get_as_ik_filtered_msg(h, ikFiltered.t, q, qDot, qDDot);
						pub_filtered.publish(msg_filtered);
						// visualize filtered!

						///TODO: Frederico, really, what is this code logic a bunch of ifs to publish the thing in all cases,,, omg, change pls

						//adding the data to the loggers
						if (isRecording())
						{
							qLogger.appendRow(pose.t,~q);
							qDotLogger.appendRow(pose.t,~qDot);
							qDDotLogger.appendRow(pose.t,~qDDot);
						}
					}
					// record
					if (isRecording())
					{
						ROS_WARN_ONCE("Recording!");
						imuLogger.appendRow(pose.t, driver->frame);//
						qRawLogger.appendRow(pose.t, ~pose.q);
					}
					previousTime = pose.t;
					previousDt = Dt;
					if(!ros::ok())
						break;
					ros::spinOnce(); //this is necessary to be able to respond to service calls!!!!!

					std_msgs::Int64 time_ik_msg;
					time_ik_msg.data = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
					time_ik_pub.publish(time_ik_msg);

					chrono::high_resolution_clock::time_point t3;
					t3 = chrono::high_resolution_clock::now();
					std_msgs::Int64 time_msg;
					time_msg.data = std::chrono::duration_cast<std::chrono::microseconds>(t3 -t1).count();
					time_pub.publish(time_msg);
					r->sleep();
				}
			} catch (std::exception& e) {
				cout << e.what() << endl;

				driver->shouldTerminate(true);
			}

			cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

			//CSVFileAdapter::write( qRawLogger, loggerFileNameIK);
			//CSVFileAdapter::write( imuLogger, loggerFileNameIMUs);

			// // store results
			// STOFileAdapter::write(
			//         qRawLogger, subjectDir + "real_time/inverse_kinematics/qRaw_imu.sto");
		}



};



