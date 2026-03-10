#ifndef VISUALIZER_SO_HEADER_FBK_26052023
#define VISUALIZER_SO_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "osrt_ros/Visualizers/dualsink_vis.h"
#include <Common/TimeSeriesTable.h>

namespace Visualizers
{

	class SoVis:public Visualizers::DualSinkVis
	{
		public:
			SoVis() {
				ROS_INFO_STREAM("instatiated SoVis");
				soLogger = new OpenSim::TimeSeriesTable;
			}
			~SoVis() {}
			void callback(const opensimrt_msgs::DualConstPtr& message);
			void callback_multi(const opensimrt_msgs::MultiMessageConstPtr& message);
			void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 
		void after_vis() {
			visualizer->tf_prefix = "so/";
			model_reset();
		}

			//here it has to be ordered!
			OpenSim::TimeSeriesTable * soLogger;
			void before_vis();
			/*virtual void before_vis()
			{
				ROS_INFO_STREAM("adding subscribers");
				ros::NodeHandle nh("~");
				sub = nh.subscribe("dual_input",1, &Visualizers::SoVis::callback, this);

				sub_filtered = nh.subscribe("dual_input_filtered",1, &Visualizers::SoVis::callback_filtered, this);
				ROS_INFO_STREAM("added subscribers ok.");
				
			}*/

			//void after_callback();
	};

}
#endif

