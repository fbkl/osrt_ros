/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : osimtfpublisher.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include "Actuators/Schutte1993Muscle_Deprecated.h"
#include <Common/Object.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Rotation.h>
#include <SimTKcommon/internal/State.h>
#include <SimTKcommon/internal/Transform.h>
#include <Simulation/Model/Model.h>
#include <Simulation/Model/ModelVisualizer.h>
#include <vector>
#include "OpenSimUtils.h"
#include "Actuators/Thelen2003Muscle.h"
#include "osrt_ros/parameters.h"
//this is a better version of human_state_publisher, aka, forward kinematics! it doesnt work though.

//since i dont know this, lets at least make it a function, so once it is known i can reuse it maybe.
//
geometry_msgs::Transform fromSimTkTransform(SimTK::Transform some_simtk_tf)
{
	geometry_msgs::Transform some_tf;	
	auto qq = some_simtk_tf.R().convertRotationToQuaternion();
	auto qt = some_simtk_tf.T();
	some_tf.rotation.w = qq[0];
	some_tf.rotation.x = qq[1];
	some_tf.rotation.y = qq[2];
	some_tf.rotation.z = qq[3];
	some_tf.translation.x = qt[0];
	some_tf.translation.y = qt[1];
	some_tf.translation.z = qt[2];
	return some_tf;
}

class Osim_tf_publisher
{
	public:
		//	we sort of want just common stuff here too, right? this is maybe a common node.
		Osim_tf_publisher()
		{
			ros::NodeHandle nh("~");
			nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
			std::string modelFile;
			nh.param<std::string>("model_file", modelFile, "/catkin_ws/src/osrt_ros/simple");
			ROS_INFO_STREAM("Using modelFile:" << modelFile);
			muscleModel = new OpenSim::Schutte1993Muscle_Deprecated();
			OpenSim::Object::RegisterType(*muscleModel);
			model.setUseVisualizer(true);
			pars::setGeometryPath(nh);
			model.setAllControllersEnabled(false);
			model = OpenSim::Model(modelFile);
			ROS_INFO_STREAM("created model okay");
			//OpenSimRT::OpenSimUtils::removeActuators(model);
			state = model.initSystem();
			ROS_INFO_STREAM("Initialized model okay");
			ros::Subscriber sync_input_sub = nh.subscribe("joint_states", 10, &Osim_tf_publisher::callback, this);
			ROS_INFO_STREAM("Initialized okay");
			nh.getParam("tf_bodies", imuBodiesObservationOrder);
			if (imuBodiesObservationOrder.size() == 0)
			{
				ROS_FATAL("IMU observation order not defined!");
				throw(std::invalid_argument("imuObservationOrder not defined."));
			}
			else
			{
				ROS_INFO_STREAM("Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
			}
			v = new SimTK::Vector(imuBodiesObservationOrder.size());
		}
		tf2_ros::TransformBroadcaster tf_broadcaster;

		//lets load a model here

		OpenSim::Model model;	
		OpenSim::Object* muscleModel;
		SimTK::State state;
		std::vector<std::string> imuBodiesObservationOrder;
		SimTK::Vector *v;

		std::string tf_frame_prefix;
		void callback(const sensor_msgs::JointStateConstPtr msg)
		{
			// creates some q to send to a model
			std::map<std::string, SimTK::Transform> imuBodiesInGround;
			if (model.isValidSystem())
			{
				for (int i=0;i<msg->position.size();i++)
					v[i] =msg->position[i];
				state.updQ() = (const SimTK::Vector)*v;	
				//model.realizePosition(state);
				//now publish the tfs, but they will be in opensim frame of reference
				for (const auto& label : imuBodiesObservationOrder) {
					const OpenSim::PhysicalFrame* frame = nullptr;
					if ((frame = model.findComponent<OpenSim::PhysicalFrame>(label))) {
						imuBodiesInGround[label] =
							frame->getTransformInGround(state); // R_GB

					}
					else
						ROS_WARN_STREAM("cannot find body" << label);
				}
				for(const auto& label : imuBodiesObservationOrder) {
					auto some_simtk_tf = imuBodiesInGround[label];
					geometry_msgs::TransformStamped some_tf;	
					some_tf.header = msg->header;
					some_tf.child_frame_id = label;
					some_tf.transform = fromSimTkTransform(some_simtk_tf);
					tf_broadcaster.sendTransform(some_tf);
				}

			}
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
			//OpenSim::Object::RegisterType(OpenSim::Thelen2003Muscle());


	//something like imuBodiesObservation, but not really
	Osim_tf_publisher op;
	//

	ros::spin();
	return 0;
}

