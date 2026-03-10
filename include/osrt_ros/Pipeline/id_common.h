#ifndef PIPELINE_COMMON_ID_HEADER_FBK_15062023
#define PIPELINE_COMMON_ID_HEADER_FBK_15062023

#include "InverseDynamics.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorBase.h>
#include <vector>
#include "osrt_ros/events.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/message_traits.h"

namespace Pipeline
{

	class IdCommon:public Pipeline::DualSink
	{
		public:
			IdCommon();
			~IdCommon();
			virtual void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik);
			virtual void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf);
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); 
			virtual void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf);
			
			virtual void run(const std_msgs::Header h, double t, std::vector<SimTK::Vector> iks, std::vector<OpenSimRT::ExternalWrench::Input> ,  opensimrt_msgs::Events e);

			void onInit();

			// now all the stuff I need to save between inInit and the callbacks

			std::string subjectDir;
			//loggers
			//
			OpenSim::TimeSeriesTable *tauLogger, *ikLogger, *grfmRightLogger, *grfmLeftLogger;
			
			//other stuff
			OpenSimRT::InverseDynamics* id;
			OpenSimRT::ForceDecorator *rightGRFDecorator,*leftGRFDecorator;
			OpenSim::Model* model;
			OpenSimRT::LowPassSmoothFilter* ikfilter;
			std::vector<std::string> grfRightLabels, grfLeftLabels;
			
			OpenSimRT::ExternalWrench::Parameters grfRightFootPar, grfLeftFootPar;
			boost::array<int,9> grfLeftIndexes, grfRightIndexes;

			bool use_grfm_filter;
			int memory, delay, splineOrder;
			double cutoffFreq;

			void finish();
			bool see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

			void write_();

			void print_vec(std::vector<std::string> vs);
			virtual void publish_additional_topics(std_msgs::Header h, SimTK::Vector q, std::vector<OpenSimRT::ExternalWrench::Input> wV)
			{ ROS_ERROR_STREAM("publish additional topics not implemented");}
			std::vector<OpenSimRT::ExternalWrench::Parameters> wrenchParameters;
	}; //end class IdCommon
} // end namespace Pipeline
#endif
