#include "osrt_ros/parameters.h"
#include "osrt_ros/Pipeline/grf_pipe.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
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
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/agrf_pipe.h"

using namespace std;
using namespace OpenSim;
;
using namespace OpenSimRT;

Pipeline::Acc::Acc()
{}
void Pipeline::Acc::onInit()
{
	Pipeline::Grf::onInit();
	get_params();

}

void Pipeline::Acc::get_params()
{
	ros::NodeHandle nh("~");
	// subject data
	//TODO: make it real params
	INIReader ini(INI_FILE);
	auto section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";
	grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", SimTK::Vec3(0));

	// acceleration-based detector parameters
	auto heelAccThreshold = ini.getReal(section, "HEEL_ACC_THRESHOLD", 0);
	auto toeAccThreshold = ini.getReal(section, "TOE_ACC_THRESHOLD", 0);
	auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
	auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
	auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
	auto rHeelLocation =
		ini.getSimtkVec(section, "RIGHT_HEEL_LOCATION_IN_FOOT", SimTK::Vec3(0));
	auto lHeelLocation =
		ini.getSimtkVec(section, "LEFT_HEEL_LOCATION_IN_FOOT", SimTK::Vec3(0));
	auto rToeLocation =
		ini.getSimtkVec(section, "RIGHT_TOE_LOCATION_IN_FOOT", SimTK::Vec3(0));
	auto lToeLocation =
		ini.getSimtkVec(section, "LEFT_TOE_LOCATION_IN_FOOT", SimTK::Vec3(0));
	auto accLPFilterFreq = ini.getInteger(section, "ACC_LP_FILTER_FREQ", 0);
	auto velLPFilterFreq = ini.getInteger(section, "VEL_LP_FILTER_FREQ", 0);
	auto posLPFilterFreq = ini.getInteger(section, "POS_LP_FILTER_FREQ", 0);
	auto accLPFilterOrder = ini.getInteger(section, "ACC_LP_FILTER_ORDER", 0);
	auto velLPFilterOrder = ini.getInteger(section, "VEL_LP_FILTER_ORDER", 0);
	auto posLPFilterOrder = ini.getInteger(section, "POS_LP_FILTER_ORDER", 0);
	auto posDiffOrder = ini.getInteger(section, "POS_DIFF_ORDER", 0);
	auto velDiffOrder = ini.getInteger(section, "VEL_DIFF_ORDER", 0);

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

	// acceleration-based event detector
	AccelerationBasedPhaseDetector::Parameters detectorParameters;
	detectorParameters.heelAccThreshold = heelAccThreshold;
	detectorParameters.toeAccThreshold = toeAccThreshold;
	detectorParameters.windowSize = windowSize;
	detectorParameters.rFootBodyName = rFootBodyName;
	detectorParameters.lFootBodyName = lFootBodyName;
	detectorParameters.rHeelLocationInFoot = rHeelLocation;
	detectorParameters.lHeelLocationInFoot = lHeelLocation;
	detectorParameters.rToeLocationInFoot = rToeLocation;
	detectorParameters.lToeLocationInFoot = lToeLocation;
	detectorParameters.samplingFrequency = 1 / 0.01;
	detectorParameters.accLPFilterFreq = accLPFilterFreq;
	detectorParameters.velLPFilterFreq = velLPFilterFreq;
	detectorParameters.posLPFilterFreq = posLPFilterFreq;
	detectorParameters.accLPFilterOrder = accLPFilterOrder;
	detectorParameters.velLPFilterOrder = velLPFilterOrder;
	detectorParameters.posLPFilterOrder = posLPFilterOrder;
	detectorParameters.posDiffOrder = posDiffOrder;
	detectorParameters.velDiffOrder = velDiffOrder;
	detector = new AccelerationBasedPhaseDetector(model, detectorParameters);

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
	counter = 0;
}

Pipeline::Acc::~Acc()
{
	ROS_INFO_STREAM("Shutting down Acc");
}


