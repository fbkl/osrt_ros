/**
 * @author      : $USER ($USER [at] cbe0486182c4)
 * @file        : parameters.cpp
 * @date     : Wednesday Jul 05, 2023 14:50:53 UTC
 */

#include "osrt_ros/parameters.h"
#include "InverseDynamics.h"
#include "MuscleOptimization.h"
#include "SignalProcessing.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "ros/node_handle.h"
#include <SimTKcommon/SmallMatrix.h>
#include <vector>

OpenSimRT::MuscleOptimization::OptimizationParameters pars::getparamSO(ros::NodeHandle nh)
{
	auto convergenceTolerance 	= nh.param<double>	("convergence_tolerance",0.);
	auto memoryHistory 		= nh.param<int>		("memory_history"	,0);
	auto maximumIterations 		= nh.param<int>		("maximum_iterations"	,0);
	auto objectiveExponent		= nh.param<int>		("objective_exponent"	,0);
	auto o_par = OpenSimRT::MuscleOptimization::OptimizationParameters();
	o_par.convergenceTolerance = convergenceTolerance;
	o_par.maximumIterations = maximumIterations;
	o_par.memoryHistory = memoryHistory;
	o_par.objectiveExponent = objectiveExponent;
	return o_par;
}
//TODO: consider using the same filter ger param for the IK filter or the GRFM or 2 functions
OpenSimRT::LowPassSmoothFilter::Parameters pars::getparamFilterIK(ros::NodeHandle nh, int num_signals)
{
	OpenSimRT::LowPassSmoothFilter::Parameters ikFilterParam;
	getparamFilterIK(nh, num_signals, ikFilterParam);
	return ikFilterParam;
};
bool pars::getparamFilterIK(ros::NodeHandle nh, int num_signals, OpenSimRT::LowPassSmoothFilter::Parameters &ikFilterParam)
{
	int memory;
	nh.param<int>("memory", memory, 0);
	double cutoffFreq; 
	nh.param<double>("cutoff_freq", cutoffFreq, 0);
	int delay;
	nh.param<int>("delay", delay, 0);
	int splineOrder;
	nh.param<int>("spline_order", splineOrder, 0);

	ROS_INFO_STREAM("FILTER PARAMS:\nmemory: " << memory << "cutoffFreq: " << cutoffFreq << "delay: " << delay << "splineOrder: " << splineOrder);
	//ikFilterParam.numSignals = model.getNumCoordinates();
	ikFilterParam.numSignals = num_signals;
	ikFilterParam.memory = memory;
	ikFilterParam.delay = delay;
	ikFilterParam.cutoffFrequency = cutoffFreq;
	ikFilterParam.splineOrder = splineOrder;
	ikFilterParam.calculateDerivatives = true;
	if (memory == 0)
	{
		ROS_WARN_STREAM("A filter with memory 0 will not work. There are other requirements for the filter, make sure those make sense, or this will not work.");
		return false;
	}
	return true;
}
OpenSimRT::LowPassSmoothFilter::Parameters pars::getparamFilterGRFM(ros::NodeHandle nh)
{
	//Params for GFRM filter
	int memory;
	nh.param<int>("memory", memory, 0);
	double cutoffFreq; 
	nh.param<double>("cutoff_freq", cutoffFreq, 0);
	int delay;
	nh.param<int>("delay", delay, 0);
	int splineOrder;
	nh.param<int>("spline_order", splineOrder, 0);
	OpenSimRT::LowPassSmoothFilter::Parameters grfFilterParam;
	grfFilterParam.numSignals = 9;
	grfFilterParam.memory = memory;
	grfFilterParam.delay = delay;
	grfFilterParam.cutoffFrequency = cutoffFreq;
	grfFilterParam.splineOrder = splineOrder;
	grfFilterParam.calculateDerivatives = false;
	return grfFilterParam;
}

std::vector<std::string> pars::getparamGRFMLabels(ros::NodeHandle nh, std::string grf_name_prefix)
{
	//TODO: the grf_right should be replaced by a string so we can have multiple GRFM inputs and reuse this
	///These only make sense for reading a file, I think so maybe they should be in bridge?
	//GFRM params
	std::string grfPointIdentifier;
	nh.param<std::string>(grf_name_prefix + "_point_identifier", grfPointIdentifier, "");
	std::string grfForceIdentifier;
	nh.param<std::string>(grf_name_prefix + "_force_identifier", grfForceIdentifier, "");
	std::string grfTorqueIdentifier;
	nh.param<std::string>(grf_name_prefix + "_torque_identifier", grfTorqueIdentifier, "");
	ROS_DEBUG_STREAM(grfPointIdentifier << " " << grfForceIdentifier << " " << grfTorqueIdentifier);
	auto res =OpenSimRT::ExternalWrench::createGRFLabelsFromIdentifiers(grfPointIdentifier, grfForceIdentifier,grfTorqueIdentifier);
	for (auto rr: res)
		ROS_DEBUG_STREAM(rr <<"; ");
	return res;
}
OpenSimRT::ExternalWrench::Parameters pars::getparamWrench(ros::NodeHandle nh, std::string grf_name_prefix)
{
	//TODO: the grf_right should be replaced by a string so we can have multiple GRFM inputs and reuse this
	///These only make sense for reading a file, I think so maybe they should be in bridge?
	//GFRM params
	std::string grfApplyBody;
	nh.param<std::string>(grf_name_prefix + "_apply_to_body", grfApplyBody, "");
	std::string grfForceExpressed;
	nh.param<std::string>(grf_name_prefix + "_force_expressed_in_body", grfForceExpressed, "");
	std::string grfPointExpressed;
	nh.param<std::string>(grf_name_prefix + "_point_expressed_in_body", grfPointExpressed, "");
	ROS_WARN_STREAM(grf_name_prefix <<
			"\ngrf_params:"<< 
			"\n\tapply body"<<grfApplyBody <<
			"\n\tforce expressed"<<grfForceExpressed <<
			"\n\tpoint expressed"<<grfPointExpressed);
	return OpenSimRT::ExternalWrench::Parameters{grfApplyBody, grfForceExpressed,  grfPointExpressed};
}

OpenSimRT::AccelerationBasedPhaseDetector::Parameters pars::getparamAccPhaseDetector(ros::NodeHandle nh) { 
	// acceleration-based event detector
	
	   OpenSimRT::AccelerationBasedPhaseDetector::Parameters detectorParameters;
	   detectorParameters.heelAccThreshold 		= nh.param<double>	("heel_acc_threshold"	,0.); //heelAccThreshold;
	   detectorParameters.toeAccThreshold 		= nh.param<double>	("toe_acc_threshold"	,0.); //oeAccThreshold;
	   detectorParameters.windowSize 		= nh.param<int>		("window_size"	,0); //indowSize;
	   detectorParameters.rFootBodyName 		= nh.param<std::string>	("r_foot_body_name"	,""); //FootBodyName;
	   detectorParameters.lFootBodyName 		= nh.param<std::string>	("l_foot_body_name"	,""); //FootBodyName;
	   detectorParameters.rHeelLocationInFoot 	= getSimtkVec(nh, 	 "r_heel_location_in_foot"	); //HeelLocation;
	   detectorParameters.lHeelLocationInFoot 	= getSimtkVec(nh, 	 "l_heel_location_in_foot"	); //HeelLocation;
	   detectorParameters.rToeLocationInFoot 	= getSimtkVec(nh, 	 "r_toe_location_in_foot"	); //ToeLocation;
	   detectorParameters.lToeLocationInFoot 	= getSimtkVec(nh, 	 "l_toe_location_in_foot"	); //ToeLocation;
	   detectorParameters.samplingFrequency 	= nh.param<double>	("sampling_frequency"	,0.); // / 0.01;
	   detectorParameters.accLPFilterFreq 		= nh.param<double>	("acc_lp_filter_freq"	,0.); //ccLPFilterFreq;
	   detectorParameters.velLPFilterFreq 		= nh.param<double>	("vel_lp_filter_freq"	,0.); //elLPFilterFreq;
	   detectorParameters.posLPFilterFreq 		= nh.param<double>	("pos_lp_filter_freq"	,0.); //osLPFilterFreq;
	   detectorParameters.accLPFilterOrder 		= nh.param<int>		("acc_lp_filter_order"	,0); //ccLPFilterOrder;
	   detectorParameters.velLPFilterOrder 		= nh.param<int>		("vel_lp_filter_order"	,0); //elLPFilterOrder;
	   detectorParameters.posLPFilterOrder 		= nh.param<int>		("pos_lp_filter_order"	,0); //osLPFilterOrder;
	   detectorParameters.posDiffOrder 		= nh.param<int>		("pos_diff_order"	,0); //osDiffOrder;
	   detectorParameters.velDiffOrder 		= nh.param<int>		("vel_diff_order"	,0); //elDiffOrder;
	   
	return detectorParameters;
}

OpenSimRT::ContactForceBasedPhaseDetector::Parameters pars::getparamConPhaseDetector(ros::NodeHandle nh) {
	//TODO:: implement!
	
	OpenSimRT::ContactForceBasedPhaseDetector::Parameters detectorParameters;
	/*   detectorParameters.threshold = threshold;
	   detectorParameters.windowSize = windowSize;
	   detectorParameters.plane_origin = SimTK::Vec3(0.0, platform_offset, 0.0);
	   detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
	   detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
	   detectorParameters.rToeSphereLocation = rToeSphereLocation;
	   detectorParameters.lToeSphereLocation = lToeSphereLocation;
	   detectorParameters.sphereRadius = contactSphereRadius;
	   detectorParameters.rFootBodyName = rFootBodyName;
	   detectorParameters.lFootBodyName = lFootBodyName;
	 * 
	 */
	return detectorParameters;

}

void pars::getSimtkVec(ros::NodeHandle nh, std::string name, SimTK::Vec3 &vec){

	std::vector<double> tempVec;
	nh.getParam(name, tempVec);
	if (tempVec.size()!=3)
		ROS_ERROR_STREAM("Cannot assign vecs of different sizes! 3 !="<< tempVec.size() );
	vec[0] = tempVec[0];
	vec[1] = tempVec[1];
	vec[2] = tempVec[2];
}

SimTK::Vec3  pars::getSimtkVec(ros::NodeHandle nh, std::string name ){
	SimTK::Vec3 vec(0);
	pars::getSimtkVec(nh, name, vec);
	return vec;
}
OpenSimRT::GRFMPrediction::Parameters pars::getparamGRFM(ros::NodeHandle nh)
{

	auto grfmMethod = nh.param<std::string>("method","");
	auto  pelvisBodyName   		= nh.param<std::string>	("pelvis_body_name"	,"");
	auto  rFootBodyName    		= nh.param<std::string>	("r_foot_body_name"	,"");
	auto  lFootBodyName    		= nh.param<std::string>	("l_foot_body_name"	,"");

	SimTK::Vec3 rHeelCoPLocation,lHeelCoPLocation, rToeCoPLocation, lToeCoPLocation;
	getSimtkVec(	nh, "r_heel_cop_location"	,	  rHeelCoPLocation );
	getSimtkVec(	nh, "l_heel_cop_location"	,	  lHeelCoPLocation );
	getSimtkVec(	nh, "r_toe_cop_location"	,	  rToeCoPLocation  );
	getSimtkVec(	nh, "l_toe_cop_location"	,	  lToeCoPLocation  );
	auto  directionWindowSize 	= nh.param<int>		("direction_window_size",0);

	OpenSimRT::GRFMPrediction::Parameters grfmParameters;
	grfmParameters.method = OpenSimRT::GRFMPrediction::selectMethod(grfmMethod);
	grfmParameters.pelvisBodyName = pelvisBodyName;
	grfmParameters.rStationBodyName = rFootBodyName;
	grfmParameters.lStationBodyName = lFootBodyName;
	grfmParameters.rHeelStationLocation = rHeelCoPLocation;
	grfmParameters.lHeelStationLocation = lHeelCoPLocation;
	grfmParameters.rToeStationLocation = rToeCoPLocation;
	grfmParameters.lToeStationLocation = lToeCoPLocation;
	grfmParameters.directionWindowSize = directionWindowSize;
	return grfmParameters;

}

void pars::setGeometryPath(ros::NodeHandle nh)
{
				std::vector<std::string> geometryPath;
				nh.getParam("geometry_path_list", geometryPath);	
				if (geometryPath.size() == 0)
				{
					std::string default_path = "/srv/data/geometry_v3.3";
					ROS_WARN_STREAM("You didn't set any paths to use for lookup of geometry meshes, using default hardcoded path of: "<<default_path);
					geometryPath.push_back(default_path);
				}

				// visualizer
				ROS_DEBUG_STREAM("Setting up visualizer");
				for (auto& a_path:geometryPath)
				{
					ROS_INFO_STREAM("Adding geometry path of" << a_path);
					OpenSim::ModelVisualizer::addDirToGeometrySearchPaths(a_path);
				}




}


