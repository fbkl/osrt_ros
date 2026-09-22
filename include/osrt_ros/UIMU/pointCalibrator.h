/**
 * @author      : $USER ($USER@frkle-Predator-PT515-52)
 * @file        : pointCalibrator.h
 * @created     : Thursday Sep 17, 2026 13:57:41 UTC
 */

#ifndef POINTCALIBRATOR_H
#define POINTCALIBRATOR_H
#include "osrt_ros/UIMU/pointGetters.h"

class PointCalibrator
{
	public:
		PointCalibrator(OpenSim::Model* model_, GetPoint* pointGetter_): 
			model(model_),
			pointGetter(pointGetter_)
		{
			// so we have the order and it is the same because it is the same freaking object, so that's cool
		



		};
		std::vector<TransObs> calibSamples;
		
		SimTK::Array_<SimTK::Vec3> mt;

		void recordNumOfSamples(const size_t& numSamples)
		{
			//eh, we also gotta store them somewhere
			// 
			for (size_t i=0 ; i<numSamples;i++) 
			{
				calibSamples.push_back(pointGetter->get_new_data()); // get_data is blocking and always gets new data
			}
		}

		void computeAvgStaticPose()
		{
			//oh god TransObs is a pair of lists of vec3, accuracy
			//and this is supposedly in the order of mmList, i think, from the pointGetter
			
			
			auto numSamples = calibSamples.size();
			auto numSensors = calibSamples[0].first.size(); 
			std::vector<size_t> counts(numSensors,numSamples);
			SimTK::Array_<SimTK::Vec3> _mt(numSensors, SimTK::Vec3(0));
			for (const auto& isamp:calibSamples)
			{
				SimTK::Array_<SimTK::Vec3> actualObservations = isamp.first;
				SimTK::Array_<SimTK::Real> accuracyOfObservations = isamp.second;
				for (int32_t i = 0; i < pointGetter->markerList.size(); ++i)  
				{
					//claude wants it weighed.. i also hope that vector scalar multiplication are correctly defined
					if (!std::isnan(accuracyOfObservations[i]))
						_mt[i]+=actualObservations[i]*accuracyOfObservations[i]; 
					else // it is a nan
						counts[i]--;
				}
			}
			for (int32_t i = 0; i < pointGetter->markerList.size(); ++i) 
			{
				if (counts[i] ==0) 
					ROS_ERROR_STREAM("marker " << pointGetter->markerNames[i] << "has zero valid observations!!!");
				else
					_mt[i]/=counts[i];
			}
			mt = _mt;
		}

	OpenSim::Model* model;
	GetPoint* pointGetter;
	//something to save the observation order maybe the state?
};

#endif /* end of include guard POINTCALIBRATOR_H */

