#include "ros/console_backend.h"
#include "ros/node_handle.h"
#include <Actuators/Thelen2003Muscle.h>

#include <ros/ros.h>

#include <exception>
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>
#include "osrt_ros/UIMU/UIMUnode.h"
#include <osrt_ros/UIMUConfig.h>
#include <osrt_ros/headingConfig.h>

int main(int argc, char** argv) {
	OpenSim::Object* muscleModel = nullptr;
	try {
		ros::init(argc, argv, "online_lower_limb_uimu_ik");
		ros::NodeHandle n;

		// DO NOT re-add a set_logger_level() call here without reading this. 🙃
		//
		// There used to be a line setting this to Fatal. It silently ate EVERY
		// ROS_INFO/WARN/ERROR from this package -- including the loud model-default-pose
		// banner in IMUCalibrator::setup() -- and cost a whole debugging session that
		// went looking at paramiko, tmux and log4cxx instead.
		//
		// The trap: ROSCONSOLE_DEFAULT_NAME is NOT the root logger. ros/console.h:302-304
		//
		//     #define ROSCONSOLE_NAME_PREFIX  "ros" "." ROSCONSOLE_PACKAGE_NAME
		//     #define ROSCONSOLE_DEFAULT_NAME ROSCONSOLE_NAME_PREFIX
		//
		// ROS_PACKAGE_NAME is a per-package compile-time macro, so in here it expands to
		// "ros.osrt_ros" -- this package only. Anything inherited from opensimrt_core logs
		// under "ros.opensimrt_core", a SIBLING logger, and is completely unaffected. That
		// asymmetry is why the node looked half-silenced rather than silenced, which is
		// exactly what made it hard to spot.
		//
		// If you ever do want to gag the whole process, the macro you want is
		// ROSCONSOLE_ROOT_LOGGER_NAME ("ros"). Leaving it unset gives the default: INFO.
		UIMUnode o;

		ros::NodeHandle nh("~");
		// either like this:
		muscleModel = new OpenSim::Thelen2003Muscle();
		o.registerType(muscleModel);
		// or alternatively, more simply:
		// Object::registerType(Thelen2003Muscle());
		bool wait_to_start = false;
		nh.getParam("wait_to_start", wait_to_start);


		if (wait_to_start)
		{
			ros::Rate wait_rate(0.1);
			bool start_now = false;
			boost::function<bool(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)> wait_callback = [&start_now](auto req, auto res) -> bool
			{
				start_now = true;
				return true;
			};
			//create a service, wait for that service call to change state so we can start
			//
			ros::ServiceServer ss = nh.advertiseService("start_now", wait_callback);

			while (!start_now)
			{
				ROS_INFO_STREAM_THROTTLE(1,"Waiting to start");
				wait_rate.sleep();
				ros::spinOnce();
			}
		}
	ros::AsyncSpinner* s;
		s = new ros::AsyncSpinner(4);
		s->start();

		o.onInit();
		o.run();
	} catch (std::exception& e) {
		cout << "Program crashed while running. Reason: " << e.what() << endl;
		return -1;
	}
	delete(muscleModel);
	return 0;
}



