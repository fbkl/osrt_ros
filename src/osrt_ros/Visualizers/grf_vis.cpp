#include "osrt_ros/Visualizers/grf_vis.h"
#include "experimental/GRFMPrediction.h"
#include "opensimrt_msgs/Dual.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include "opensimrt_bridge/conversions/message_convs.h"
#include "osrt_ros/Visualizers/dualsink_vis.h"

using opensimrt_msgs::DualConstPtr;

void Visualizers::GrfVis::before_vis()
			{
				ROS_WARN("removing actuators and initializing system here too. i shouldnt do this more than once, so please move this logic so no more copy pastes,, okay");

				OpenSimRT::OpenSimUtils::removeActuators(*model);

				model->initSystem();
				Visualizers::DualSinkVis::before_vis();
			}
void Visualizers::GrfVis::after_vis()
{
	ROS_INFO("grf after_vis called!!");
	bool modelOk = true;
	rightGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Blue, 0.001, 3);
	nh.param<std::string>("grf_right_point", right_body_name, "");
	if (!right_body_name.empty())
	{
		rightGRFDecorator->setOriginByName(*model, right_body_name);
		visualizer->addDecorationGenerator(rightGRFDecorator);
	}
	else
	{
		ROS_ERROR_STREAM("grf right application point not set, not adding decorator!");
		modelOk = false;
	}
	

	leftGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Red, 0.001, 3);
	nh.param<std::string>("grf_left_point", left_body_name, "");
	if (!left_body_name.empty())
	{
		leftGRFDecorator->setOriginByName(*model, left_body_name);
		visualizer->addDecorationGenerator(leftGRFDecorator);
	}
	else
	{
		ROS_ERROR_STREAM("grf left application point not set! not adding decorator");
		modelOk = false;
	}
	model->initSystem();
	if (modelOk)
	{
		ROS_INFO("I think model is okay, trying to refresh visuals");
		visualizer->refreshModel();
		OpenSimRT::GRFMPrediction::Output grfmOutput; //hopefully starts with zeros everywhere
		grfmOutput.left.point[0] = 0.0;
		grfmOutput.left.point[1] = 0.0;
		grfmOutput.left.point[2] = -0.01;
		grfmOutput.left.force[0] = 10.0;
		grfmOutput.left.force[1] = 1000.0;
		grfmOutput.left.force[2] = 10.0;
		grfmOutput.right.point[0] = 0.0;
		grfmOutput.right.point[1] = 0.0;
		grfmOutput.right.point[2] = 0.01;
		grfmOutput.right.force[0] = 10.0;
		grfmOutput.right.force[1] = 1000.0;
		grfmOutput.right.force[2] = 10.0;
		rightGRFDecorator->update(grfmOutput.right.point, grfmOutput.right.force);
		leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);
	}
	Visualizers::VisualizerCommon::after_vis();
}

void Visualizers::GrfVis::callback(const DualConstPtr &msg) {
	ROS_ERROR_STREAM("not implemented");
	//get q from message
	SimTK::Vector q(msg->q.data.size());
	for (size_t i=0;i<msg->q.data.size(); i++)
	{
		q[i] = msg->q.data[i];
	}
	//get grfs from message
	OpenSimRT::GRFMPrediction::Output grfmOutput;
	
	try {
		visualizer->update(q);
		// after update q
		ROS_DEBUG_STREAM("updated visuals ok");
		rightGRFDecorator->update(grfmOutput.right.point, grfmOutput.right.force);
		leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
	} catch (std::exception &e) {
		ROS_ERROR_STREAM_ONCE("Error in visualizer. cannot show data!!!!!" << std::endl
				<< e.what());
	}
}

void Visualizers::GrfVis::callback_filtered(const opensimrt_msgs::DualPosConstPtr & msg)
{
	ROS_ERROR_STREAM("not implemented");

}
