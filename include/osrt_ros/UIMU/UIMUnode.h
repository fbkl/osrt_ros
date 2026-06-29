#include "XmlRpcException.h"
#include "XmlRpcValue.h"
#include "geometry_msgs/TransformStamped.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/UIMU/IMUCalibrator.h"
//#include "INIReader.h"
#include "InverseKinematics.h"
#include "SignalProcessing.h"
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
//#include <OpenSim/Common/STOFileAdapter.h>

#include "osrt_ros/parameters.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/PoseStamped.h"
#include "opensimrt_msgs/Labels.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Array.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/ReinitOnCopy.h>
#include <chrono>
#include <exception>
#include <string>
#include <unordered_map>
#include <vector>
#include "osrt_ros/UIMU/TfServer.h"
#include "Ros/include/common_node.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

#include <dynamic_reconfigure/server.h>
#include <osrt_ros/UIMUConfig.h>
#include <osrt_ros/headingConfig.h>
#include "osrt_ros/events.h"
#include "opensimrt_bridge/conversions/message_convs.h"

#include "osrt_ros/UIMU/pointGetters.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

class UIMUnode: Ros::CommonNode
{
	public:
		UIMUnode(): Ros::CommonNode(false), pointGetter(nullptr), ikfilter(nullptr) //if true debugs
			    //UIMUnode(): Ros::CommonNode()
	{}
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg1 =0.0, yGroundRotDeg1 = 0.0, zGroundRotDeg1 = 0.0;
		std::vector<std::string> imuObservationOrder;
		double rate = 100.0;
		ros::Rate* r = nullptr;
		std::string modelFile;
		std::string loggerFileNameIK,loggerFileNameIMUs;
		std::string tf_frame_prefix;
		double sumDelayMS = 0, numFrames = 0;
		double previousTime = 0;
		double previousDt = 0;
		OpenSim::TimeSeriesTable imuLogger, imuCalibrationLogger, qRawLogger, qLogger, qDotLogger, qDDotLogger;

		ros::Publisher time_pub, time_ik_pub;
		ros::ServiceServer calibrationService;
		UIMUInputDriver *driver = nullptr;
		InverseKinematics * ik = nullptr;
		IMUCalibrator * clb = nullptr;
		bool clb_is_ready =false;
		bool usePositionMarkers = false;
		bool useOrientationMarkers = true;
		bool visualiseIt= false;
		//ros::Publisher re_pub;
		//filter parameters
		double cutoffFreq = 0.0;
		int splineOrder = 3, memory = 0, delay = 0 ;
		OpenSimRT::LowPassSmoothFilter * ikfilter;
		std::vector<ros::Publisher> plottable_outputs;
		//dynamic_reconfigure::Server<osrt_ros::UIMUConfig> server;
		//dynamic_reconfigure::Server<osrt_ros::UIMUConfig>::CallbackType f;
		OpenSim::Model model;

		GetPoint* pointGetter;

		void get_params();
		void registerType(Object* muscleModel); //do I even need this?
		void reconfigure_callback(osrt_ros::UIMUConfig &config, uint32_t level);
		void reconfigure_heading_callback(osrt_ros::headingConfig &config, uint32_t level);
		vector<InverseKinematics::MarkerTask> markerTasks;
		vector<InverseKinematics::IMUTask> imuTasks;
		void define_tasks();
		void start_ik();

		SimTK::RowVector fromVectorOfSimTKQuaternionsToARowVector(std::vector<SimTK::Quaternion> vv);
		void clearLogger(TimeSeriesTable &t); //TODO: move it somewhere nice. maybe make loggers a wrapper class
		void doCalibrate();
		bool calibrationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		void onInit();

		double last_time = -1.1;
		void run();


};



