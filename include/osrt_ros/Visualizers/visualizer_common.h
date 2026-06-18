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
			OpenSim::Object* muscleModel;
			ros::ServiceServer resetModelSrv;
			
			bool resetModel(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
			{
			
				model_reset();
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

				resetModelSrv = nh.advertiseService("reset", &VisualizerCommon::resetModel, this);

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
			void model_reset()
			{
				int i =0;
				SimTK::Vector q(input.labels.size());
				for (auto label:input.labels)
				{
					q[i]=0.0; // to show like a model, make it nicer
					i++;
				}
				visualizer->update(q);



			}

			virtual void after_vis()
			{
				ROS_WARN_ONCE("Not implemented for VisualizerCommon. post-setup of the thing");
				model_reset();
			}

			virtual void after_callback()
			{
				//ROS_WARN("Not implemented. does something after the callbacks.");
			}
			void callback(const opensimrt_msgs::CommonTimedConstPtr &msg_ik ) {
				try { // main loop
				      // unroll
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
