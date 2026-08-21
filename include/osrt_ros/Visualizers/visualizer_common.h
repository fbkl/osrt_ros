#ifndef VISUALIZER_COMMON_HEADER_FBK_27052023
#define VISUALIZER_COMMON_HEADER_FBK_27052023

#include "Ros/include/common_node.h"
#include "Ros/include/saver_node.h"
#include "Settings.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include "OpenSimUtils.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/Object.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <exception>
#include "osrt_ros/parameters.h"

#include "ros/service_client.h"
#include "ros/service_server.h"
#include "std_srvs/Empty.h"
//TODO:this is rather bad and I should be able to load models using some string input
//TODO: remove this switch statement, find something better.
//
namespace Visualizers
{
	class VisualizerCommon:public Ros::SaverNode
	{
		public:
			VisualizerCommon()
			{
			}
			ros::NodeHandle nh{"~"};
			std::string modelFile;
			OpenSimRT::ModelObserver *visualizer=nullptr;
			OpenSim::Model* model = nullptr;	
			int m; // 1 is upper 2 is under...
			bool visualiseIt= false;
			ros::Subscriber sub, sub_filtered;
			Ros::Reshuffler input;
			SimTK::Vector defaultQ; // the model's authored default pose, captured once at onInit
			OpenSim::Object* muscleModel;
			ros::ServiceServer poseDefaultSrv, poseZeroSrv, resetModelSrv;
			std::string vis_name;

			// "reset" was ambiguous: it meant zero, but the model's baseline is its
			// authored default pose (what initSystem() gives, and what IMUCalibrator
			// assumes). Name the pose, not the operation.
			bool poseModelDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
			{

				pose_model_default();
				return true;

			}

			bool poseModelZero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
			{

				pose_model_zero();
				return true;

			}

			[[deprecated("'reset' was ambiguous and used to mean ZERO; it now poses at the model DEFAULT. bind pose_model_default() or pose_model_zero() explicitly and delete this alias.")]]
			bool resetModel(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
			{

				ROS_WARN_ONCE("service [reset] is deprecated and now poses the model at its DEFAULT values, not zero. use [pose_default] or [pose_zero].");
				pose_model_default();
				return true;

			}

			void set_delay_from_header(ros::Time t)
			{
				if (visualizer)
					visualizer->fps->actual_delay = (ros::Time::now().toSec() - t.toSec())*1000; // this is in ms

			}
			void get_params()
			{
				// subject data
				nh.param<std::string>("model_file", modelFile, "");
				ROS_INFO_STREAM("Using modelFile:" << modelFile);
				nh.param<int>("which_model_1_2", m, 2);
				ROS_DEBUG_STREAM("Finished getting params.");	
				
				nh.param<std::string>("vis_name", vis_name, "");

			}
			void registerType(OpenSim::Object* muscleModel) //do I even need this?
			{
				OpenSim::Object::registerType(*muscleModel);
			}
			void onInit() 
			{
				get_params();
				model = new OpenSim::Model(modelFile);

				input.model = model;
				Ros::SaverNode::onInit();
				// setup model
				input.get_labels(nh);
				ROS_DEBUG_STREAM("Setting up model.");

				poseDefaultSrv = nh.advertiseService("pose_default", &VisualizerCommon::poseModelDefault, this);
				poseZeroSrv = nh.advertiseService("pose_zero", &VisualizerCommon::poseModelZero, this);
				resetModelSrv = nh.advertiseService("reset", &VisualizerCommon::resetModel, this); // deprecated alias, now DEFAULT not zero

				switch(m)
				{
					case 1:
						muscleModel = new OpenSim::Schutte1993Muscle_Deprecated();
						registerType(muscleModel);
						break;
					case 2:
						muscleModel = new OpenSim::Thelen2003Muscle();
						registerType(muscleModel);
						break;
					default:
						throw std::invalid_argument( "I can use 1, upper or 2, lower. this is hardcoded." );
				}
				
					

				pars::setGeometryPath(nh);
				before_vis();
				if (!model->isValidSystem())
				{
					ROS_WARN_STREAM("model is not valid yet for some reason, trying to make it valid");
					model->initSystem();
				}
				// Capture the model's authored default pose ONCE, straight from the system.
				// Deliberately independent of input.labels: the default pose is a property of
				// the MODEL, so it must not depend on the Reshuffler handshake having happened,
				// on the remaps being written correctly, or on label order. Note also that
				// ModelObserver::update does `state.updQ() = q`, and Q is in MULTIBODY-TREE
				// order, which is NOT CoordinateSet order -- so a name-keyed lookup over
				// input.labels would be silently wrong even when the handshake did work.
				defaultQ = model->initSystem().getQ();
				nh.param<bool>("visualise", visualiseIt, true);
				if (visualiseIt)
				{
					ROS_DEBUG_STREAM("Setting up visualizer");
					visualizer = new OpenSimRT::BasicModelVisualizer(*model);
				}
				else
				{
					ROS_DEBUG_STREAM("Setting up model observer");
					visualizer = new OpenSimRT::ModelObserver(*model);

				}
				visualizer->setVisualizer();
				visualizer->publish_transforms = true;
				
				OpenSimRT::BasicModelVisualizer* vvv = dynamic_cast<OpenSimRT::BasicModelVisualizer*>(visualizer);
				if (vvv != nullptr){
				std::string actualTitle = model->getName().empty() ? "<unnamed>" : model->getName() ;
				actualTitle+= " " +  sub.getTopic() + " " + vis_name;
				auto& viz = vvv->visualizer;
				viz->setWindowTitle(actualTitle);
				}
				after_vis();	

				ROS_DEBUG_STREAM("onInit finished just fine.");
			}
			virtual void before_vis()
			{
				ROS_INFO("called before vis of visualizer common, removing actuators and initializing system");
				OpenSimRT::OpenSimUtils::removeActuators(*model);
				model->initSystem();
				ROS_WARN_ONCE("Not implemented for VisualizerCommon. initial setup of the thing");
			}
			// Poses the model for display. use_defaults=true uses each coordinate's authored
			// default_value -- what initSystem() gives, and what IMUCalibrator::setup() assumes
			// when it captures imuBodiesInGround. false forces zero, which is a DEBUGGING pose
			// and agrees with the calibration only for models whose defaults are all zero.
			// (MOBL_ARMS has r_y = -90 deg on the ground-attached groundthorax joint: zero here
			// meant the visualiser and the calibration silently disagreed by exactly that yaw.)
			void pose_model(bool use_defaults)
			{
				// NO input.labels here, on purpose -- see defaultQ's capture in onInit().
				if (!visualizer)
				{
					ROS_ERROR("cannot pose the model: no visualizer/observer exists. setup bug.");
					return;
				}
				if (defaultQ.size() == 0)
				{
					ROS_ERROR("cannot pose the model: the default pose was never captured, so the model system was not initialised at onInit(). refusing to publish a pose rather than publishing a wrong one.");
					return;
				}
				SimTK::Vector q(defaultQ);
				if (!use_defaults)
					q.setToZero();
				visualizer->update(q);
			}
			void pose_model_default() { pose_model(true); }
			void pose_model_zero() { pose_model(false); }

			virtual void after_vis()
			{
				ROS_WARN_ONCE("Not implemented for VisualizerCommon. post-setup of the thing");
				pose_model_default();
			}

			virtual void after_callback()
			{
				//ROS_WARN("Not implemented. does something after the callbacks.");
			}
			void callback(const opensimrt_msgs::CommonTimedConstPtr &msg_ik ) {
				try { // main loop
				      // unroll
				        if(msg_ik->data.size()==0) {
						ROS_FATAL_THROTTLE(1,"No data in topic!");
					}
					SimTK::Vector q(msg_ik->data.size());
					for (size_t i=0;i<msg_ik->data.size();i++)
					{
						q[i] = msg_ik->data[i];
					}
					visualizer->update(q);
					set_delay_from_header(msg_ik->header.stamp);
					after_callback();
				} catch (std::exception& e) {
					std::cout << e.what() << std::endl;
				}
			}
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr &msg_ik ) {
				try { // main loop filtered
				      // unroll
				        if(msg_ik->d0_data.size()==0) {
						ROS_FATAL_THROTTLE(1,"No data in filtered topic!");
					}
					SimTK::Vector q(msg_ik->d0_data.size());
					for (size_t i=0;i<msg_ik->d0_data.size();i++)
					{
						q[i] = msg_ik->d0_data[i];
					}
					visualizer->update(q);
					set_delay_from_header(msg_ik->header.stamp);
					after_callback();

				} catch (std::exception& e) {
					std::cout << e.what() << std::endl;
				}
			}
	};
}
#endif
