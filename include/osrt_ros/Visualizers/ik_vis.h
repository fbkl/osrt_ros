#ifndef VISUALIZER_IK_HEADER_FBK_27052023
#define VISUALIZER_IK_HEADER_FBK_27052023
#include "OpenSimUtils.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Visualizers/visualizer_common.h"
#include "ros/node_handle.h"

namespace Visualizers
{
	class IkVis:public Visualizers::VisualizerCommon
	{
		std::string own_tf_prefix;
		void callback(const opensimrt_msgs::CommonTimedConstPtr &msg_ik ); 
		void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr &msg_ik );
		void before_vis() {
			OpenSimRT::OpenSimUtils::removeActuators(*model);
			//Subscribers.
			sub = nh.subscribe<opensimrt_msgs::CommonTimed>("input", 1 ,&Visualizers::IkVis::callback, this);
			sub_filtered = nh.subscribe<opensimrt_msgs::PosVelAccTimed>("input_filtered",1, &Visualizers::IkVis::callback_filtered, this);

		};
		void after_vis() {

			nh.param<std::string>("own_tf_prefix", own_tf_prefix, "ik/");
			visualizer->tf_prefix = own_tf_prefix;
			model_reset();
		}
		void after_callback() {};
	};

}

#endif
