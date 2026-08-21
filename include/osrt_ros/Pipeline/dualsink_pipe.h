#ifndef PIPELINE_DUALSINK_HEADER_FBK_21072022
#define PIPELINE_DUALSINK_HEADER_FBK_21072022

// NAG -- deliberate, do not silence without deleting the thing it is nagging about.
//
// The dual-sink pattern exists because every sink/source is its own hardcoded class, which
// in turn exists because label ordering is primed out-of-band (Reshuffler + a service
// handshake) instead of being carried by the data. That priming is the debt: it buys
// microseconds nobody measures and costs a whole class of silent failures (handshake never
// happened, remap written wrong, labels empty -> node does nothing and says nothing), plus
// it makes a single node impossible to test by replaying a bag into it.
//
// NOT the debt: the custom time sequencing. ROS1 message_filters ApproximateTime genuinely
// falls over on the ~300ms-lagged insole stream, so that workaround is load-bearing.
//
// Plan: carry labels with the data (latched labels topic, or in-message), decode with a
// cached lookup, then sinks/sources stop needing to be classes and can just be composed.
// Do it with the ROS2 port. See first_paperino.md.
#pragma message("osrt_ros: dualsink/Reshuffler label-priming is TECH DEBT scheduled for removal -- see first_paperino.md")

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "opensimrt_msgs/MultiMessagePosVelAcc.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_srvs/Empty.h"
#include "Ros/include/common_node.h"
#include "opensimrt_msgs/PosVelAccTimed.h"

namespace Pipeline
{
	class DualSink:public Ros::CommonNode
	{
		public:

			DualSink(bool debug=false);
			~DualSink();
			bool get_second_label = true;
			//this is a sink, I think. should be made into a class, but I think it will be complicated, so I didnt do it.
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub2; //input2
										       //ros::Subscriber sub2; //input2
			message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync;
			message_filters::TimeSynchronizer<opensimrt_msgs::PosVelAccTimed, opensimrt_msgs::CommonTimed> sync_filtered;
			std::vector<std::string> input2_labels;
			void onInit();
			ros::Subscriber sync_input_sub, sync_input_filtered_sub;
			[[deprecated]] ros::Publisher sync_output, sync_output_filtered;
			ros::Publisher sync_output_multi, sync_output_multi_filtered;
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message, const opensimrt_msgs::CommonTimedConstPtr& message2) 
			{
				ROS_ERROR_STREAM("dual message callback not implemented!");
			}
			virtual void callback1(const opensimrt_msgs::CommonTimedConstPtr& message)
			{
				ROS_ERROR_STREAM("callback should not have been registered by itself. testing individual message callback1");
			}
			virtual void callback2(const opensimrt_msgs::CommonTimedConstPtr& message)
			{
				ROS_ERROR_STREAM("callback should not have been registered by itself. testing individual message callback2");
			}
			virtual void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message, const opensimrt_msgs::CommonTimedConstPtr& message2) 
			{
				ROS_ERROR_STREAM("dual message callback_filtered not implemented!");
			}
			virtual void sync_callback(const opensimrt_msgs::MultiMessageConstPtr& message) 
			{
				ROS_ERROR_STREAM("MultiMessage message callback not implemented!");
			}
			virtual void sync_callback_filtered(const opensimrt_msgs::MultiMessagePosVelAccConstPtr& message) 
			{
				ROS_ERROR_STREAM("MultiMessagePosVelAcc message callback_filtered not implemented!");
			}
	};
}
#endif

