#ifndef VISUALIZER_ID_HEADER_FBK_27052023
#define VISUALIZER_ID_HEADER_FBK_27052023
#include "OpenSimUtils.h"
#include "osrt_ros/Visualizers/grf_vis.h"

namespace Visualizers
{
	class IdVis:public Visualizers::GrfVis
	{
		
		void callback_multi(const opensimrt_msgs::MultiMessageConstPtr& message);
		void after_callback() {};
		void after_vis() {
			visualizer->tf_prefix = "id/";
			model_reset();
		}
	};

}


#endif
