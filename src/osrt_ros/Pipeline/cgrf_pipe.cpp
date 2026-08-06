#include "osrt_ros/parameters.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/cgrf_pipe.h"

using namespace std;
using namespace OpenSim;
;
using namespace OpenSimRT;


Pipeline::Fc::Fc()
{}
void Pipeline::Fc::onInit()
{
	Pipeline::Grf::onInit();
	get_params();
	
}

void Pipeline::Fc::get_params()
{
	ros::NodeHandle nh("~");
	// subject data
	//TODO: make real params
	INIReader ini(INI_FILE);
	auto section = "TEST_CONTACT_FORCE_GRFM_PREDICTION_FROM_FILE";

	grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", SimTK::Vec3(0));

	// virtual contact surface as ground
	auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

	// acceleration-based detector parameters
	auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
	auto threshold = ini.getReal(section, "THRESHOLD", 0);

	auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
	auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");

	auto rHeelSphereLocation =
		    ini.getSimtkVec(section, "RIGHT_HEEL_SPHERE_LOCATION", SimTK::Vec3(0));
	auto lHeelSphereLocation =
		    ini.getSimtkVec(section, "LEFT_HEEL_SPHERE_LOCATION", SimTK::Vec3(0));
	auto rToeSphereLocation =
		    ini.getSimtkVec(section, "RIGHT_TOE_SPHERE_LOCATION", SimTK::Vec3(0));
	auto lToeSphereLocation =
		    ini.getSimtkVec(section, "LEFT_TOE_SPHERE_LOCATION", SimTK::Vec3(0));
	auto contactSphereRadius = ini.getReal(section, "SPHERE_RADIUS", 0);



	// setup model
	Object::RegisterType(Thelen2003Muscle());
	OpenSim::Model model(modelFile);
	model.initSystem();
	
	// setup external forces parameters
	ExternalWrench::Parameters grfRightFootPar = pars::getparamWrench(nh, "grf_right");
	auto grfRightLabels = pars::getparamGRFMLabels(nh, "grf_right");
	auto grfRightLoggerTemp = ExternalWrench::initializeLogger();
	grfRightLogger = &grfRightLoggerTemp;
	ExternalWrench::Parameters grfLeftFootPar = pars::getparamWrench(nh, "grf_left");
	auto grfLeftLabels = pars::getparamGRFMLabels(nh, "grf_left");
	auto grfLeftLoggerTemp = ExternalWrench::initializeLogger();
	grfLeftLogger = &grfLeftLoggerTemp;
	
	output.labels.insert(output.labels.end(),grfRightLabels.begin(),grfRightLabels.end());
	output.labels.insert(output.labels.end(),grfLeftLabels.begin(),grfLeftLabels.end());
	

	vector<ExternalWrench::Parameters> wrenchParameters;
	wrenchParameters.push_back(grfRightFootPar);
	wrenchParameters.push_back(grfLeftFootPar);

	// setup filters
	LowPassSmoothFilter::Parameters filterParam;
	if (pars::getparamFilterIK(nh, model.getNumCoordinates(), filterParam))
		filter = new LowPassSmoothFilter(filterParam);
	else
		ROS_WARN_STREAM("IK Filter not created, if used this will crash.");

	// contact force based event detection
	ContactForceBasedPhaseDetector::Parameters detectorParameters;
	detectorParameters.threshold = threshold;
	detectorParameters.windowSize = windowSize;
	detectorParameters.plane_origin = SimTK::Vec3(0.0, platform_offset, 0.0);
	detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
	detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
	detectorParameters.rToeSphereLocation = rToeSphereLocation;
	detectorParameters.lToeSphereLocation = lToeSphereLocation;
	detectorParameters.sphereRadius = contactSphereRadius;
	detectorParameters.rFootBodyName = rFootBodyName;
	detectorParameters.lFootBodyName = lFootBodyName;
	detector = new ContactForceBasedPhaseDetector(model, detectorParameters);

	auto grfmParameters = pars::getparamGRFM(nh);
	grfm = new GRFMPrediction(model, grfmParameters, detector);

	// id
	id = new InverseDynamics(model, wrenchParameters);
	auto tauLoggerTemp = id->initializeLogger();
	tauLogger = &tauLoggerTemp;

	// visualizer
	visualizer = new BasicModelVisualizer(model);
	visualizer->setVisualizer();
	rightGRFDecorator = new ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(rightGRFDecorator);
	leftGRFDecorator = new ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(leftGRFDecorator);

	// mean delay
	sumDelayMS = 0;
	sumDelayMSCounter = 0;

	//loopCounter = 0;
	//i = 0;
}

Pipeline::Fc::~Fc()
{
	ROS_INFO_STREAM("Shutting down Fc.");
}

