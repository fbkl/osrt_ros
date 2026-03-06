/**
 * @author      : $USER (osruser [at]cbe0486182c4)
 * @file        : parameters.h
 * @date     : Wednesday Jul 05, 2023 14:39:41 UTC
 */

//this has to be moved to opensimrt_core so that bridge nodes can also use it. 

#ifndef PARAMETERS070523_H

#define PARAMETERS070523_H
#include "MuscleOptimization.h"
#include "SignalProcessing.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "ros/node_handle.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include <vector>

/*
 * @brief I don't know why I thought it was a good idea to put the parameter getter definitions in a class. 
 *
 * TODO: rethink if you want this.
 */
namespace pars
{
	/*
	 * Gets ros parameters for an SO node
	 */
	OpenSimRT::MuscleOptimization::OptimizationParameters getparamSO(ros::NodeHandle nh);
	/*
	 * Gets parameters to instantiate a low pass filter
	 */
	OpenSimRT::LowPassSmoothFilter::Parameters 		getparamFilterIK	(ros::NodeHandle nh, int num_signals);
	bool 							getparamFilterIK	(ros::NodeHandle nh, int num_signals, OpenSimRT::LowPassSmoothFilter::Parameters &par);
	OpenSimRT::LowPassSmoothFilter::Parameters 		getparamFilterGRFM	(ros::NodeHandle nh);
	/*
	 * Gets the point, force and torque string parameters to read a CommonTimed message that was created from like a table opensim object 
	 */
	std::vector<std::string> 				getparamGRFMLabels	(ros::NodeHandle nh, std::string grf_name_prefix);
	OpenSimRT::ExternalWrench::Parameters 			getparamWrench		(ros::NodeHandle nh, std::string grf_name_prefix);
	OpenSimRT::AccelerationBasedPhaseDetector::Parameters 	getparamAccPhaseDetector(ros::NodeHandle nh);
	OpenSimRT::ContactForceBasedPhaseDetector::Parameters 	getparamConPhaseDetector(ros::NodeHandle nh);
	OpenSimRT::GRFMPrediction::Parameters 			getparamGRFM		(ros::NodeHandle nh);

	void getSimtkVec(ros::NodeHandle nh, std::string name, SimTK::Vec3 &vec);
	SimTK::Vec3 getSimtkVec(ros::NodeHandle nh, std::string name);
	void setGeometryPath(ros::NodeHandle nh);
	}
#endif /* end of include guard PARAMETERS070523_H */

