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

		void recordNumOfSamples(const size_t& numSamples)
		{
			//eh, we also gotta store them somewhere
			// 
			for (size_t i=0 ; i<numSamples;i++) 
			{
				calibSamples.push_back(pointGetter->get_translations());
				//some sort of waiting
			}
		}

		void computeAvgStaticPose()
		{
			//oh god TransObs is a pair of lists of vec3, accuracy
			//and this is supposedly in the order of mmList, i think, from the pointGetter
		}

	OpenSim::Model* model;
	GetPoint* pointGetter;
	//something to save the observation order maybe the state?
};

#endif /* end of include guard POINTCALIBRATOR_H */

